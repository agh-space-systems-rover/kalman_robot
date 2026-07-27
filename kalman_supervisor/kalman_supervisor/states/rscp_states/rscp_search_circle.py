import numpy as np
import time

from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterType, Parameter
from std_srvs.srv import Trigger

# Radius of the search circle. Matches the outermost radius of the stage 1
# spiral (revolution width 2 * 3 revolutions = 6 m).
CIRCLE_RADIUS = 3.0
MIN_DISTANCE_TO_GOAL = 1.0
PROGRESS_INCREMENT = 0.001


class RSCPSearchCircle(State):
    def __init__(self):
        super().__init__("rscp_search_circle")

    def circle(self, progress: float) -> np.ndarray:
        # progress 0..1 maps to one full revolution; the path starts and ends
        # at the robot's entry point.
        t = self.start_angle + 2 * np.pi * progress
        return self.center + CIRCLE_RADIUS * np.array([np.cos(t), np.sin(t)])

    def circle_as_msg(self) -> Path:
        msg = Path()
        msg.header.frame_id = self.supervisor.tf.world_frame()
        msg.header.stamp = self.supervisor.get_clock().now().to_msg()
        for i in range(100):
            progress = i / 99
            xy = self.circle(progress)
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = xy[0]
            pose.pose.position.y = xy[1]
            msg.poses.append(pose)
        return msg

    def clear_elevation_map(self) -> None:
        req = Trigger.Request()
        self.clear_elevation_map_client.call_async(req)

    def toggle_follower_slow_approach(self, enabled: bool) -> None:
        req = SetParameters.Request()
        param = Parameter()
        param.name = f"approach_distance"
        param.value.type = ParameterType.PARAMETER_DOUBLE
        param.value.double_value = (
            self.default_follower_approach_distance if enabled else 0.5
        )
        req.parameters.append(param)
        self.follower_set_params.call_async(req)
        self.slow_approach_enabled = enabled

    def fetch_default_follower_approach_distance(self) -> float:
        req = GetParameters.Request()
        req.names = ["approach_distance"]
        future = self.follower_get_params.call_async(req)

        def callback(future):
            res: GetParameters.Response = future.result()
            self.default_follower_approach_distance = res.values[0].double_value

        future.add_done_callback(callback)

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Starting circle...")

        self.clear_elevation_map_client = self.supervisor.create_client(
            Trigger, "/peak_finder_node/clear_elevation_map"
        )
        self.circle_pub = self.supervisor.create_publisher(
            Path, "supervisor/circle", 10
        )

        stage = self.supervisor.rscp.get_current_stage()
        if stage != 1:
            self.supervisor.get_logger().warn(
                f"[RSCP] Stage is not 1, current is {stage} while in SearchCircle"
            )
        # Stage 1 logic: start peak mapping from scratch.
        self.clear_elevation_map()

        # Init the circle. The center is placed to the left of the robot so
        # the circle passes through the entry point and the robot drives it
        # counter-clockwise, finishing where it started.
        entry_robot_pos = self.supervisor.tf.robot_pos()[:2]
        entry_robot_rot = self.supervisor.tf.robot_rot_2d()
        self.center = entry_robot_pos + CIRCLE_RADIUS * np.array(
            [np.cos(entry_robot_rot + np.pi / 2), np.sin(entry_robot_rot + np.pi / 2)]
        )
        # Angle from the center to the entry point.
        self.start_angle = entry_robot_rot - np.pi / 2

        # Start slightly ahead so the first goal is not the robot's own position.
        self.init_progress = MIN_DISTANCE_TO_GOAL / (2 * np.pi * CIRCLE_RADIUS)
        self.progress = self.init_progress
        self.last_circle_goal_time = 0

        # Publish circle for debugging.
        self.circle_pub.publish(self.circle_as_msg())

        # Disable slow approach in path follower.
        self.follower_set_params = self.supervisor.create_client(
            SetParameters, "search/path_follower/set_parameters"
        )
        self.follower_get_params = self.supervisor.create_client(
            GetParameters, "search/path_follower/get_parameters"
        )
        self.default_follower_approach_distance: float | None = None
        self.fetch_default_follower_approach_distance()
        self.slow_approach_enabled = True
        self.next_goal_timeout = 0.0

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            self.supervisor.get_logger().warn(
                "[RSCP] DISARM detected during navigation, aborting"
            )
            self.supervisor.rscp.clear_search_goal()
            return "rscp_idle"

        # Until progress reaches 1, keep sending circle goals.
        # Send goal if:
        # - there is no goal
        # - the distance to goal is less than MIN_DISTANCE_TO_GOAL meters
        # - next goal timeout has passed
        now = time.time()
        if self.progress < 1 and (
            not self.supervisor.nav.has_goal()
            or self.supervisor.nav.distance_to_goal() < MIN_DISTANCE_TO_GOAL
            or now - self.last_circle_goal_time > self.next_goal_timeout
        ):
            old_goal = self.circle(self.progress)
            goal = old_goal
            if self.progress != self.init_progress:
                while (
                    self.progress < 1
                    and np.linalg.norm(goal - old_goal) < MIN_DISTANCE_TO_GOAL
                ):
                    self.progress += PROGRESS_INCREMENT
                    goal = self.circle(self.progress)

            self.next_goal_timeout = (
                30.0 if self.progress == self.init_progress else 10.0
            )

            self.supervisor.nav.send_goal(np.append(goal, 0))
            self.progress += PROGRESS_INCREMENT
            self.last_circle_goal_time = now

        # Once we have the default approach distance, disable slow approach.
        if (
            self.default_follower_approach_distance is not None
            and self.slow_approach_enabled
        ):
            self.supervisor.get_logger().info(
                "[Search] Disabling slow approach in path follower."
            )
            self.toggle_follower_slow_approach(False)

        # The circle is complete once the full revolution was commanded and
        # the final goal (the starting point) has been reached.
        if self.progress >= 1.0 and not self.supervisor.nav.has_goal():
            self.supervisor.get_logger().info(
                "[RSCP] Circle complete, transitioning to rscp_navigate_peak"
            )
            return "rscp_navigate_peak"

    def exit(self) -> None:
        # Reset slow approach in path follower.
        if (
            self.default_follower_approach_distance is not None
            and not self.slow_approach_enabled
        ):
            self.supervisor.get_logger().info(
                "[Search] Re-enabling slow approach in path follower."
            )
            self.toggle_follower_slow_approach(True)

        self.supervisor.destroy_publisher(self.circle_pub)

        if self.supervisor.nav.has_goal():
            self.supervisor.get_logger().info(
                "[RSCP] Exiting rscp_search_circle, canceling navigation"
            )
            self.supervisor.nav.cancel_goal()
