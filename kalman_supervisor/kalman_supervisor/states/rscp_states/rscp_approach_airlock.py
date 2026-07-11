import time
import numpy as np
import math

from kalman_supervisor.state import State
from kalman_supervisor.modules import *

WORKING_FRAME = "odom"
GATE_MARKER_ID = 297
APPROACH_DISTANCE = 1.5
PASS_DURATION = 7.5
PASS_SPEED = 0.3
SEARCH_ROTATION_SPEED = 0.3
MAX_ROTATE_TO_GATE_SPEED = 0.3


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def get_airlock_context(supervisor):
    if not hasattr(supervisor, "airlock_context"):
        supervisor.airlock_context = {
            "gate_mid": None,
            "gate_vec": None,
            "entry_pos": None,
            "gate_mid_map": None,
            "gate_vec_map": None,
            "entry_pos_map": None,
        }
    return supervisor.airlock_context


class RscpWaitBeforeAirlock(State):
    def __init__(self):
        super().__init__("rscp_wait_before_airlock")

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Airlock: waiting before search...")
        self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.ARMED)
        self.start_time = time.time()

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        if time.time() - self.start_time >= 2.0:
            return "rscp_airlock_search_gate"

        return None


class RscpAirlockSearchGate(State):
    def __init__(self):
        super().__init__("rscp_airlock_search_gate")

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Airlock: searching for gate...")
        self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.AUTONOMOUS)
        self.supervisor.aruco.enable()

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        gate = self.supervisor.aruco.gate_pos(GATE_MARKER_ID, WORKING_FRAME)
        gate_map = self.supervisor.aruco.gate_pos(GATE_MARKER_ID, "map")

        if gate is None or gate_map is None:
            self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, SEARCH_ROTATION_SPEED)
            return None

        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
        self.supervisor.get_logger().info("[RSCP] Airlock: gate found.")

        if self.calc_gate_entry_to_ctx(gate, WORKING_FRAME) is None:
            self.supervisor.get_logger().warn("[RSCP] Airlock: gate width too small.")
            return None

        if self.calc_gate_entry_to_ctx(gate_map, "map", "_map") is None:
            self.supervisor.get_logger().warn("[RSCP] Airlock: map gate width too small.")
            return None

        ctx = get_airlock_context(self.supervisor)
        M = ctx["gate_mid"]
        E = ctx["entry_pos"]
        M_map = ctx["gate_mid_map"]

        self.supervisor.get_logger().info(
            f"[RSCP] Airlock: gate mid: {M}, entry: {E}, mid_map: {M_map}"
        )

        return "rscp_airlock_navigate_entry"

    def exit(self) -> None:
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)

    def calc_gate_entry_to_ctx(self, gate: np.ndarray, frame: str, suffix: str = ""):
        p1, p2 = gate
        p1_2d, p2_2d = p1[:2], p2[:2]

        gate_mid = (p1_2d + p2_2d) / 2.0
        v = p2_2d - p1_2d
        v_len = np.linalg.norm(v)

        if v_len < 0.1:
            return None

        v_dir = v / v_len
        gate_vec = np.array([-v_dir[1], v_dir[0]])

        robot_pos = self.supervisor.tf.robot_pos(frame)[:2]
        if np.dot(gate_vec, gate_mid - robot_pos) < 0:
            gate_vec = -gate_vec

        entry_pos = gate_mid - APPROACH_DISTANCE * gate_vec

        ctx = get_airlock_context(self.supervisor)
        ctx["gate_mid" + suffix] = gate_mid
        ctx["gate_vec" + suffix] = gate_vec
        ctx["entry_pos" + suffix] = entry_pos

        return entry_pos


class RscpAirlockNavigateEntry(State):
	def __init__(self):
		super().__init__("rscp_airlock_navigate_entry")
		self.goal_threshold = 0.2
		self.max_linear_speed = 0.3
		self.k_p = 1.0

	def enter(self) -> None:
		ctx = get_airlock_context(self.supervisor)
		self.goal_pos = ctx["entry_pos"]
		self.supervisor.get_logger().info(
			f"[RSCP] Airlock: navigating to entry position: {self.goal_pos}"
		)

	def tick(self) -> str | None:
		if not self.supervisor.rscp.is_armed():
			return "rscp_idle"

		# convert 2d goal to 3d for the api
		goal_3d = np.append(self.goal_pos, 0.0)
		
		# transform goal from world/working frame to robot frame
		goal_bf = self.supervisor.tf.transform_numpy(
			goal_3d, self.supervisor.tf.robot_frame(), WORKING_FRAME
		)
		
		# calculate distance in 2d
		distance = np.linalg.norm(goal_bf[:2])

		if distance < self.goal_threshold:
			self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
			self.supervisor.get_logger().info("[RSCP] Airlock: reached entry pos.")
			return "rscp_airlock_rotate_to_gate"

		# calculate velocity components in robot-centric frame
		direction_unit = goal_bf[:2] / distance
		speed = min(self.max_linear_speed, distance * self.k_p)
		
		vx = speed * direction_unit[0]
		vy = speed * direction_unit[1]
		
		self.supervisor.cmd_vel.send_cmd_vel(vx, vy, 0.0)
		return None

	def exit(self) -> None:
		self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)


class RscpAirlockRotateToGate(State):
    def __init__(self):
        super().__init__("rscp_airlock_rotate_to_gate")

    def enter(self) -> None:
        ctx = get_airlock_context(self.supervisor)
        gate_vec = ctx.get("gate_vec_map")
        
        if gate_vec is not None:
            self.target_yaw = math.atan2(gate_vec[1], gate_vec[0])
        else:
            self.target_yaw = self.supervisor.tf.robot_rot_2d("map")
            self.supervisor.get_logger().warn("[RSCP] Gate vector not found! Will not rotate.")

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        yaw = self.supervisor.tf.robot_rot_2d("map")
        diff = normalize_angle(self.target_yaw - yaw)

        if abs(diff) < 0.1:
            return "rscp_airlock_drive_through"

        vel = 0.2 * np.sign(diff) + 0.2 * diff
        vel = max(-MAX_ROTATE_TO_GATE_SPEED, min(MAX_ROTATE_TO_GATE_SPEED, vel))
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, vel)
        return None

    def exit(self) -> None:
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)


class RscpAirlockDriveThrough(State):
    def __init__(self):
        super().__init__("rscp_airlock_drive_through")

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Airlock: driving through.")
        self.start_time = time.time()
        self.supervisor.aruco.disable()

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        if time.time() - self.start_time < PASS_DURATION:
            self.supervisor.cmd_vel.send_cmd_vel(PASS_SPEED, 0.0, 0.0)
            return None

        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
        self.supervisor.get_logger().info("[RSCP] Airlock completed.")
        return "rscp_idle"

    def exit(self) -> None:
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
