import numpy as np
import time
import math
from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterType, Parameter

WORKING_FRAME = "map"
GATE_MARKER_ID = 269
GOAL_DISTANCE = 4.0
ENTRY_DISTANCE = 1.0
UPDATE_THRESHOLD = 2.0
MAX_DISTANCE = 20.0
CEILING_STOP_BUFFER = 5.0
ARUCO_STOP_MIN_DISTANCE_TRAVELED = 7.0
ARUCO_STOP_DETECTION_TIMEOUT = 2.0
ARUCO_TIMEOUT = 2.0
SEARCH_ROTATION_SPEED = 0.3
MAX_ROTATE_TO_GATE_SPEED = 0.3


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def get_lavatube_context(supervisor):
    if not hasattr(supervisor, "lavatube_context"):
        supervisor.lavatube_context = {
            "gate_mid": None,
            "gate_vec": None,
            "entry_pos": None,
        }
    return supervisor.lavatube_context


class RscpWaitBeforeLavatube(State):
    def __init__(self):
        super().__init__("rscp_wait_before_lavatube")

    def enter(self) -> None:
        self.supervisor.get_logger().info(
            "[RSCP] Lava tube: waiting for START_EXPLORATION before search..."
        )
        self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.ARMED)

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        if self.supervisor.rscp.exploration_start_requested():
            self.supervisor.rscp.clear_exploration_start_request()
            return "rscp_lavatube_search_gate"

        return None


class RscpLavatubeSearchGate(State):
    def __init__(self):
        super().__init__("rscp_lavatube_search_gate")

    def enter(self) -> None:
        self.supervisor.get_logger().info(
            "[RSCP] Searching for lava tube gate (ID 269)"
        )
        self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.AUTONOMOUS)
        self.supervisor.aruco.enable()
        self.start_time = time.time()

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        gate = self.supervisor.aruco.gate_pos(GATE_MARKER_ID, WORKING_FRAME)
        if gate is not None:
            self.calc_gate_entry_to_ctx(gate)
            self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
            self.supervisor.get_logger().info(f"[RSCP] Gate found, moving to entry")
            return "rscp_lavatube_navigate_entry"

        # Timeout search after full rotation (approx 21 seconds at 0.3 rad/s)
        if time.time() - self.start_time > 25.0:
            self.supervisor.get_logger().warn("[RSCP] Gate not found after rotation")
            return "rscp_idle"

        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, SEARCH_ROTATION_SPEED)
        return None

    def exit(self) -> None:
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
        
    def calc_gate_entry_to_ctx(self, gate: np.ndarray):
        p1, p2 = gate
        p1_2d, p2_2d = p1[:2], p2[:2]

        gate_mid = (p1_2d + p2_2d) / 2.0
        v = p2_2d - p1_2d
        v_len = np.linalg.norm(v)

        if v_len < 0.1:
            return None

        v_dir = v / v_len
        # gate_vec is the normal vector perpendicular to the gate plane
        gate_vec = np.array([-v_dir[1], v_dir[0]])

        # Ensure vector points into the tube (away from robot)
        robot_pos = self.supervisor.tf.robot_pos(WORKING_FRAME)[:2]
        if np.dot(gate_vec, gate_mid - robot_pos) < 0:
            gate_vec = -gate_vec

        entry_pos = gate_mid - ENTRY_DISTANCE * gate_vec

        ctx = get_lavatube_context(self.supervisor)
        ctx["gate_mid"] = gate_mid
        ctx["gate_vec"] = gate_vec
        ctx["entry_pos"] = entry_pos



class RscpLavatubeNavigateEntry(State):
    def __init__(self):
        super().__init__("rscp_lavatube_navigate_entry")

    def enter(self) -> None:
        ctx = get_lavatube_context(self.supervisor)
        pos = ctx["entry_pos"]
        self.supervisor.nav.send_goal(np.append(pos, 0.0), WORKING_FRAME)

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        if not self.supervisor.nav.has_goal():
            return "rscp_lavatube_rotate_to_gate"

        return None

    def exit(self) -> None:
        if self.supervisor.nav.has_goal():
            self.supervisor.nav.cancel_goal()


class RscpLavatubeRotateToGate(State):
    def __init__(self):
        super().__init__("rscp_lavatube_rotate_to_gate")

    def enter(self) -> None:
        ctx = get_lavatube_context(self.supervisor)
        gate = self.supervisor.aruco.gate_pos(GATE_MARKER_ID, WORKING_FRAME)

        if gate is not None:
            p1, p2 = gate
            ctx["gate_mid"] = (p1[:2] + p2[:2]) / 2.0

        M = ctx["gate_mid"]
        if M is not None:
            robot_pos = self.supervisor.tf.robot_pos(WORKING_FRAME)[:2]
            to_gate = M - robot_pos
            self.target_yaw = math.atan2(to_gate[1], to_gate[0])
        else:
            self.target_yaw = self.supervisor.tf.robot_rot_2d(WORKING_FRAME)

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        yaw = self.supervisor.tf.robot_rot_2d(WORKING_FRAME)
        diff = normalize_angle(self.target_yaw - yaw)

        if abs(diff) < 0.1:
            return "rscp_lavatube_explore"

        vel = 0.5 * np.sign(diff) + 0.1 * diff
        vel = max(-MAX_ROTATE_TO_GATE_SPEED, min(MAX_ROTATE_TO_GATE_SPEED, vel))
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, vel)
        return None

    def exit(self) -> None:
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)


# class RscpLavatubeExplore(State):
#     def __init__(self):
#         super().__init__("rscp_lavatube_explore")
#         self.default_follower_approach_distance: float | None = None
#         self.slow_approach_enabled = True

#     def toggle_follower_slow_approach(self, enabled: bool) -> None:
#         req = SetParameters.Request()
#         param = Parameter()
#         param.name = "approach_distance"
#         param.value.type = ParameterType.PARAMETER_DOUBLE
#         param.value.double_value = (
#             self.default_follower_approach_distance if enabled else 0.5
#         )
#         req.parameters.append(param)
#         self.follower_set_params.call_async(req)
#         self.slow_approach_enabled = enabled

#     def fetch_default_follower_approach_distance(self) -> None:
#         req = GetParameters.Request()
#         req.names = ["approach_distance"]
#         future = self.follower_get_params.call_async(req)

#         def callback(future):
#             res: GetParameters.Response = future.result()
#             self.default_follower_approach_distance = res.values[0].double_value

#         future.add_done_callback(callback)

#     def enter(self) -> None:
#         ctx = get_lavatube_context(self.supervisor)
#         self.start_dist = self.supervisor.arc.get_distance_traveled()
#         self.last_landmark_dist = self.start_dist
#         self.last_landmark_pos = self.supervisor.tf.robot_pos(WORKING_FRAME)[:2]

#         self.total_dist_under_ceiling = 0.0
#         self.ceiling_entered_dist = None
#         self.ceiling_lost_dist = None
#         self.detected_ceiling_at_least_once = False

#         # Initialize parameter clients.
#         self.follower_set_params = self.supervisor.create_client(
#             SetParameters, "search/path_follower/set_parameters"
#         )
#         self.follower_get_params = self.supervisor.create_client(
#             GetParameters, "search/path_follower/get_parameters"
#         )
#         self.fetch_default_follower_approach_distance()

#         gate_mid = ctx["gate_mid"]
#         gate_vec = ctx["gate_vec"]
#         initial_goal = gate_mid + GOAL_DISTANCE * gate_vec
#         self.supervisor.nav.send_goal(np.append(initial_goal, 0.0), WORKING_FRAME)

#         self.supervisor.get_logger().info("[RSCP] Entering lava tube")

#     def tick(self) -> str | None:
#         if not self.supervisor.rscp.is_armed():
#             return "rscp_idle"

#         # Handle slow approach toggle once default param is fetched.
#         if (
#             self.default_follower_approach_distance is not None
#             and self.slow_approach_enabled
#         ):
#             self.toggle_follower_slow_approach(False)

#         now_dist = self.supervisor.arc.get_distance_traveled()
#         traveled = now_dist - self.start_dist
#         current_pos_3d = self.supervisor.tf.robot_pos(WORKING_FRAME)
#         current_pos = current_pos_3d[:2]

#         if self.supervisor.arc.sees_ceiling():
#             self.detected_ceiling_at_least_once = True
#             if self.ceiling_entered_dist is None:
#                 self.ceiling_entered_dist = traveled
#             self.ceiling_lost_dist = None
#         else:
#             if self.ceiling_entered_dist is not None:
#                 self.total_dist_under_ceiling += traveled - self.ceiling_entered_dist
#                 self.ceiling_entered_dist = None
#                 self.ceiling_lost_dist = traveled
#             elif self.ceiling_lost_dist is None:
#                 self.ceiling_lost_dist = traveled

#         if traveled > ARUCO_STOP_MIN_DISTANCE_TRAVELED:
#             if (
#                 self.supervisor.aruco.marker_pos(GATE_MARKER_ID, WORKING_FRAME, ARUCO_STOP_DETECTION_TIMEOUT)
#                 is not None
#             ):
#                 self.supervisor.get_logger().info("[RSCP] Aruco marker detected during exploration, stopping")
#                 return self.finish(traveled)

#         if traveled >= MAX_DISTANCE:
#             self.supervisor.get_logger().info("[RSCP] Max distance traveled in lava tube, stopping")
#             return self.finish(traveled)

#         if (
#             self.detected_ceiling_at_least_once
#             and self.ceiling_lost_dist is not None
#             and (traveled - self.ceiling_lost_dist) >= CEILING_STOP_BUFFER
#         ):
#             self.supervisor.get_logger().info("[RSCP] Lost ceiling for too long, stopping")
#             return self.finish(traveled)

#         if (now_dist - self.last_landmark_dist) >= UPDATE_THRESHOLD:
#             direction = current_pos - self.last_landmark_pos
#             dist_since_last = np.linalg.norm(direction)

#             if dist_since_last > 0.1:
#                 direction /= dist_since_last
#                 new_goal = current_pos + GOAL_DISTANCE * direction
#                 self.supervisor.nav.send_goal(np.append(new_goal, 0.0), WORKING_FRAME)

#                 self.last_landmark_dist = now_dist
#                 self.last_landmark_pos = current_pos

#         return None

#     def exit(self) -> None:
#         # Re-enable slow approach if it was disabled.
#         if (
#             self.default_follower_approach_distance is not None
#             and not self.slow_approach_enabled
#         ):
#             self.toggle_follower_slow_approach(True)

#     def finish(self, traveled: float) -> str:
#         if self.ceiling_entered_dist is not None:
#             self.total_dist_under_ceiling += traveled - self.ceiling_entered_dist
#             self.ceiling_entered_dist = None
#         if self.supervisor.nav.has_goal():
#             self.supervisor.nav.cancel_goal()
#         self.supervisor.rscp.send_distance(self.total_dist_under_ceiling)
#         self.supervisor.rscp.send_task_finished()
#         self.supervisor.aruco.disable()
#         return "rscp_idle"

class RscpLavatubeExplore(State):
	def __init__(self):
		super().__init__("rscp_lavatube_explore")
		self.default_follower_approach_distance: float | None = None
		self.slow_approach_enabled = True

	def toggle_follower_slow_approach(self, enabled: bool) -> None:
		req = SetParameters.Request()
		param = Parameter()
		param.name = "approach_distance"
		param.value.type = ParameterType.PARAMETER_DOUBLE
		param.value.double_value = (
			self.default_follower_approach_distance if enabled else 0.5
		)
		req.parameters.append(param)
		self.follower_set_params.call_async(req)
		self.slow_approach_enabled = enabled

	def fetch_default_follower_approach_distance(self) -> None:
		req = GetParameters.Request()
		req.names = ["approach_distance"]
		future = self.follower_get_params.call_async(req)

		def callback(future):
			res: GetParameters.Response = future.result()
			self.default_follower_approach_distance = res.values[0].double_value

		future.add_done_callback(callback)

	def enter(self) -> None:
		self.start_dist = self.supervisor.arc.get_distance_traveled()
		self.total_dist_under_ceiling = 0.0
		self.ceiling_entered_dist = None
		self.ceiling_lost_dist = None
		self.detected_ceiling_at_least_once = False

		# use simple supervisor tunnel follower instead of nav goals
		self.supervisor.arc.toggle_tunnel_follower(True)

		self.follower_set_params = self.supervisor.create_client(
			SetParameters, "search/path_follower/set_parameters"
		)
		self.follower_get_params = self.supervisor.create_client(
			GetParameters, "search/path_follower/get_parameters"
		)
		self.fetch_default_follower_approach_distance()
		self.supervisor.get_logger().info("[RSCP] Entering lava tube via tunnel follower")

	def tick(self) -> str | None:
		if not self.supervisor.rscp.is_armed():
			return "rscp_idle"

		if (
			self.default_follower_approach_distance is not None
			and self.slow_approach_enabled
		):
			self.toggle_follower_slow_approach(False)

		now_dist = self.supervisor.arc.get_distance_traveled()
		traveled = now_dist - self.start_dist

		if self.supervisor.arc.sees_ceiling():
			self.detected_ceiling_at_least_once = True
			if self.ceiling_entered_dist is None:
				self.ceiling_entered_dist = traveled
			self.ceiling_lost_dist = None
		else:
			if self.ceiling_entered_dist is not None:
				self.total_dist_under_ceiling += traveled - self.ceiling_entered_dist
				self.ceiling_entered_dist = None
				self.ceiling_lost_dist = traveled
			elif self.ceiling_lost_dist is None:
				self.ceiling_lost_dist = traveled

		if traveled > ARUCO_STOP_MIN_DISTANCE_TRAVELED:
			if (
				self.supervisor.aruco.marker_pos(GATE_MARKER_ID, WORKING_FRAME, ARUCO_STOP_DETECTION_TIMEOUT)
				is not None
			):
				self.supervisor.get_logger().info("[RSCP] Aruco marker detected, stopping")
				return self.finish(traveled)

		if traveled >= MAX_DISTANCE:
			self.supervisor.get_logger().info("[RSCP] Max distance reached, stopping")
			return self.finish(traveled)

		if (
			self.detected_ceiling_at_least_once
			and self.ceiling_lost_dist is not None
			and (traveled - self.ceiling_lost_dist) >= CEILING_STOP_BUFFER
		):
			self.supervisor.get_logger().info("[RSCP] Lost ceiling for too long, stopping")
			return self.finish(traveled)

		return None

	def exit(self) -> None:
		self.supervisor.arc.toggle_tunnel_follower(False)
		if (
			self.default_follower_approach_distance is not None
			and not self.slow_approach_enabled
		):
			self.toggle_follower_slow_approach(True)

	def finish(self, traveled: float) -> str:
		self.supervisor.arc.toggle_tunnel_follower(False)
		if self.ceiling_entered_dist is not None:
			self.total_dist_under_ceiling += traveled - self.ceiling_entered_dist
			self.ceiling_entered_dist = None
		self.supervisor.rscp.send_distance(self.total_dist_under_ceiling)
		self.supervisor.rscp.send_task_finished()
		self.supervisor.aruco.disable()
		return "rscp_idle"
