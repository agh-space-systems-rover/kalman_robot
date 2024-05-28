import numpy as np
import utm

from kalman_supervisor.state import State, disable_state
from kalman_supervisor.modules import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# the distance between the revolutions of the spiral
SPIRAL_REVOLUTION_WIDTH = 3
MIN_DISTANCE_TO_GOAL = 1.0
PROGRESS_INCREMENT = 0.001

def spiral_tr(progress: float, angle: float, revolutions: int) -> np.ndarray:
    t = 2 * np.pi * revolutions * progress + angle
    r = revolutions * SPIRAL_REVOLUTION_WIDTH * progress
    return (t, r)

def spiral(progress: float, angle: float, revolutions: int) -> np.ndarray:
    t, r = spiral_tr(progress, angle, revolutions)
    return np.array([r * np.cos(t), r * np.sin(t)])

@disable_state
class Search(State):
    def __init__(self, name: str, revolutions: int):
        super().__init__(name)
        self.revolutions = revolutions
        self.init_progress = 0.5 / revolutions

    def spiral_as_msg(self) -> Path:
        msg = Path()
        msg.header.frame_id = self.supervisor.tf.world_frame()
        msg.header.stamp = self.supervisor.get_clock().now().to_msg()
        for i in range(100):
            progress = i / 99
            xy = spiral(progress, self.angle, self.revolutions) + self.center
            pose = PoseStamped()
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = msg.header.stamp
            pose.pose.position.x = xy[0]
            pose.pose.position.y = xy[1]
            msg.poses.append(pose)
        return msg
    
    def reset_spiral(self, angle_offset: float) -> None:
        self.progress = self.init_progress
        self.center = self.entry_robot_pos
        self.angle = self.entry_robot_rot + angle_offset
        
        init_angle, _ = spiral_tr(self.init_progress, 0, self.revolutions)
        self.angle -= init_angle + np.pi / 2
        self.center -= spiral(self.init_progress, self.angle, self.revolutions)

        # Publish spiral for debugging.
        self.spiral_pub.publish(self.spiral_as_msg())

    def enter(self) -> None:
        # Enable the detection node.
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            if isinstance(mission, Missions.GpsArUcoSearch):
                self.supervisor.aruco.enable()
            elif isinstance(mission, Missions.GpsYoloSearch):
                self.supervisor.yolo.enable()

        # Publish spiral for debugging.
        self.spiral_pub = self.supervisor.create_publisher(Path, "supervisor/spiral", 10)

        # Init the spiral.
        self.entry_robot_pos = self.supervisor.tf.robot_pos()[:2]
        self.entry_robot_rot = self.supervisor.tf.robot_rot_2d()
        self.reset_spiral(0)

    def tick(self) -> str | None:
        # Cancel the navigation if missions was ended early.
        if not self.supervisor.missions.has_mission():
            self.supervisor.nav.cancel_goal()
            return "stop_to_teleop"

        # Until progress reaches 1, keep sending spiral goals.
        # Send goal if:
        # - there is no goal
        # - the distance to goal is less than MIN_DISTANCE_TO_GOAL meters
        if self.progress < 1 and (
            not self.supervisor.nav.has_goal()
            or self.supervisor.nav.distance_to_goal() < MIN_DISTANCE_TO_GOAL
        ):
            old_offset = spiral(self.progress, self.angle, self.revolutions)
            offset = old_offset
            if self.progress != self.init_progress:
                while (
                    self.progress < 1
                    and np.linalg.norm(offset - old_offset) < MIN_DISTANCE_TO_GOAL
                ):
                    self.progress += PROGRESS_INCREMENT
                    offset = spiral(self.progress, self.angle, self.revolutions)
            goal = self.center + offset

            self.supervisor.nav.send_goal(np.append(goal, 0))
            self.progress += PROGRESS_INCREMENT

        # Check if the searched item was found and transition to approach.
        mission = self.supervisor.missions.get_mission()
        if isinstance(mission, Missions.GpsArUcoSearch):
            marker_pos = self.supervisor.aruco.marker_pos(mission.marker_id, "utm")
            if marker_pos is not None:
                mission.marker_found = True

                zone_number = utm.latlon_to_zone_number(
                    mission.init_lat, mission.init_lon
                )
                zone_letter = utm.latitude_to_zone_letter(mission.init_lat)
                lat, lon = utm.to_latlon(
                    marker_pos[0], marker_pos[1], zone_number, zone_letter
                )

                mission.marker_lat = lat
                mission.marker_lon = lon
                mission.marker_world_frame_pos = self.supervisor.aruco.marker_pos(mission.marker_id)

                return "approach"
        elif isinstance(mission, Missions.GpsYoloSearch):
            class_pos = self.supervisor.yolo.class_pos(mission.obj_class, "utm")
            if class_pos is not None:
                mission.obj_found = True

                zone_number = utm.latlon_to_zone_number(
                    mission.init_lat, mission.init_lon
                )
                zone_letter = utm.latitude_to_zone_letter(mission.init_lat)
                lat, lon = utm.to_latlon(
                    class_pos[0], class_pos[1], zone_number, zone_letter
                )

                mission.obj_lat = lat
                mission.obj_lon = lon
                mission.obj_world_frame_pos = self.supervisor.yolo.class_pos(mission.obj_class)

                return "approach"

        # If progress reached 1 and the searched item was not found, reset the progress.
        if self.progress >= 1:
            self.supervisor.get_logger().info("Finished the spiral and failed to find the target. Returning to the center of the spiral to retry it...")
            self.reset_spiral(np.random.uniform(-np.pi, np.pi))

    def exit(self) -> None:
        self.supervisor.destroy_publisher(self.spiral_pub)        

        # Disable the detection node.
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            if isinstance(mission, Missions.GpsArUcoSearch):
                self.supervisor.aruco.disable()
            elif isinstance(mission, Missions.GpsYoloSearch):
                self.supervisor.yolo.disable()


class SearchForArUco(Search):
    def __init__(self):
        super().__init__("search_for_aruco", 7)


class SearchForYolo(Search):
    def __init__(self):
        super().__init__("search_for_yolo", 4)
