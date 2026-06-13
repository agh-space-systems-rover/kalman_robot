import numpy as np
import utm
import time

from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import ParameterType, Parameter
from std_srvs.srv import Trigger

 # the distance between the revolutions of the spiral
SPIRAL_REVOLUTION_WIDTH = 5
MIN_DISTANCE_TO_GOAL = 1.0
PROGRESS_INCREMENT = 0.001


class RSCPSearchSpiral(State):
    def __init__(self):
        super().__init__("rscp_search_spiral")
        revolutions = 2
        self.revolutions = revolutions
        self.init_progress = 0.5 / revolutions
        self.flip_spiral = True  # switched to False on first reset
        self.clear_elevation_map_client = self.supervisor.create_client(
            Trigger, "/clear_elevation_map"
        )

    def spiral_tr(self, progress: float, angle: float, revolutions: int) -> np.ndarray:
        p = progress if not self.flip_spiral else -progress
        t = 2 * np.pi * revolutions * p + angle
        r = revolutions * SPIRAL_REVOLUTION_WIDTH * abs(p)
        return (t, r)

    def spiral(self, progress: float, angle: float, revolutions: int) -> np.ndarray:
        t, r = self.spiral_tr(progress, angle, revolutions)
        return np.array([r * np.cos(t), r * np.sin(t)])

    def spiral_as_msg(self) -> Path:
        msg = Path()
        msg.header.frame_id = self.supervisor.tf.world_frame()
        msg.header.stamp = self.supervisor.get_clock().now().to_msg()
        for i in range(100):
            progress = i / 99
            xy = self.spiral(progress, self.angle, self.revolutions) + self.center
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
        # Enable the detection node.
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            if isinstance(mission, Missions.GpsArUcoSearch):
                self.supervisor.aruco.enable()
            elif isinstance(mission, Missions.GpsYoloSearch):
                self.supervisor.yolo.enable()

        # Publish spiral for debugging.
        self.spiral_pub = self.supervisor.create_publisher(
            Path, "supervisor/spiral", 10
        )

        # Init the spiral.
        self.entry_robot_pos = self.supervisor.tf.robot_pos()[:2]
        self.entry_robot_rot = self.supervisor.tf.robot_rot_2d()
        self.reset_spiral(0)

        # Disable slow approach in path follower.
        self.follower_set_params = self.supervisor.create_client(
            SetParameters, "search/path_follower/set_parameters"
        )
        self.follower_get_params = self.supervisor.create_client(
            GetParameters, "search/path_follower/get_parameters"
        )
        self.default_follower_approach_distance: float | None = None
        self.fetch_default_follower_approach_distance()
        self.clear_elevation_map()
        self.slow_approach_enabled = True
        self.next_goal_timeout = 0.0

    def tick(self) -> str | None:
        # Cancel the navigation if missions was ended early.
        if not self.supervisor.missions.has_mission():
            self.supervisor.nav.cancel_goal()
            return "stop_to_teleop"

        # Until progress reaches 1, keep sending spiral goals.
        # Send goal if:
        # - there is no goal
        # - the distance to goal is less than MIN_DISTANCE_TO_GOAL meters
        # - next goal timeout has passed
        now = time.time()
        if self.progress < 1 and (
            not self.supervisor.nav.has_goal()
            or self.supervisor.nav.distance_to_goal() < MIN_DISTANCE_TO_GOAL
            or now - self.last_spiral_goal_time > self.next_goal_timeout
        ):
            old_offset = self.spiral(self.progress, self.angle, self.revolutions)
            offset = old_offset
            if self.progress != self.init_progress:
                while (
                    self.progress < 1
                    and np.linalg.norm(offset - old_offset) < MIN_DISTANCE_TO_GOAL
                ):
                    self.progress += PROGRESS_INCREMENT
                    offset = self.spiral(self.progress, self.angle, self.revolutions)
            goal = self.center + offset

            self.next_goal_timeout = (
                30.0 if self.progress == self.init_progress else 10.0
            )

            self.supervisor.nav.send_goal(np.append(goal, 0))
            self.progress += PROGRESS_INCREMENT
            self.last_spiral_goal_time = now

        # Check if the searched item was found and transition to approach.
        mission = self.supervisor.missions.get_mission()
                
        # Once we have the default approach distance, disable slow approach.
        if (
            self.default_follower_approach_distance is not None
            and self.slow_approach_enabled
        ):
            self.supervisor.get_logger().info(
                "[Search] Disabling slow approach in path follower."
            )
            self.toggle_follower_slow_approach(False)

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

        self.supervisor.destroy_publisher(self.spiral_pub)

        
