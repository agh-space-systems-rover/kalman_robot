import numpy as np
import utm

from kalman_supervisor.state import State, disable_state
from kalman_supervisor.modules import *

# number of revolutions of the spiral per 1 progress unit
SPIRAL_REVOLUTIONS = 4
# the distance between the revolutions of the spiral
SPIRAL_REVOLUTION_WIDTH = 5
MIN_DISTANCE_TO_GOAL = 3.0
INIT_PROGRESS = 0.0
PROGRESS_INCREMENT = 0.001
ANGLE_OFFSET = 0.0

def spiral(progress: float, angle = 0.0) -> np.ndarray:
    t = 2 * np.pi * SPIRAL_REVOLUTIONS * progress + angle
    r = SPIRAL_REVOLUTIONS * SPIRAL_REVOLUTION_WIDTH * progress

    return np.array([r * np.cos(t), r * np.sin(t)])

@disable_state
class Search(State):
    def __init__(self, name: str):
        super().__init__(name)

    def enter(self) -> None:
        self.progress = INIT_PROGRESS
        self.center = self.supervisor.tf.robot_pos()[:2]
        self.angle = self.supervisor.tf.robot_rot_2d() + ANGLE_OFFSET
        
        # Enable the detection node.
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            if isinstance(mission, Missions.GpsArUcoSearch):
                self.supervisor.aruco.enable()
            elif isinstance(mission, Missions.GpsYoloSearch):
                self.supervisor.yolo.enable()

    def tick(self) -> str | None:
        # Cancel the navigation if missions was ended early.
        if not self.supervisor.missions.has_mission():
            self.supervisor.nav.cancel_goal()
            return "stop_to_teleop"

        # Until progress reaches 1, keep sending spiral goals.
        # Send goal if:
        # - there is no goal
        # - the distance to goal is less than MIN_DISTANCE_TO_GOAL meters
        if self.progress < 1 and (not self.supervisor.nav.has_goal() or self.supervisor.nav.distance_to_goal() < MIN_DISTANCE_TO_GOAL):
            old_offset = spiral(np.sqrt(self.progress), self.angle)
            offset = old_offset
            while self.progress < 1 and np.linalg.norm(offset - old_offset) < MIN_DISTANCE_TO_GOAL:
                self.progress += PROGRESS_INCREMENT
                offset = spiral(np.sqrt(self.progress), self.angle)
            goal = self.center + offset
            
            self.supervisor.nav.send_goal(np.append(goal, 0))
            self.progress += PROGRESS_INCREMENT

        # Check if the searched item was found and transition to approach.
        mission = self.supervisor.missions.get_mission()
        if isinstance(mission, Missions.GpsArUcoSearch):
            marker_pos = self.supervisor.aruco.marker_pos(mission.marker_id, "utm")
            if marker_pos is not None:
                mission.marker_found = True

                zone_number = utm.latlon_to_zone_number(mission.init_lat, mission.init_lon)
                zone_letter = utm.latitude_to_zone_letter(mission.init_lat)
                lat, lon = utm.to_latlon(marker_pos[0], marker_pos[1], zone_number, zone_letter)

                mission.marker_lat = lat
                mission.marker_lon = lon

                return "approach"
        elif isinstance(mission, Missions.GpsYoloSearch):
            class_pos = self.supervisor.yolo.class_pos(mission.obj_class, "utm")
            if class_pos is not None:
                mission.obj_found = True

                zone_number = utm.latlon_to_zone_number(mission.init_lat, mission.init_lon)
                zone_letter = utm.latitude_to_zone_letter(mission.init_lat)
                lat, lon = utm.to_latlon(class_pos[0], class_pos[1], zone_number, zone_letter)

                mission.obj_lat = lat
                mission.obj_lon = lon

                return "approach"
            
        # If progress reached 1 and the searched item was not found, reset the progress.
        if self.progress >= 1:
            self.progress = INIT_PROGRESS

    def exit(self) -> None:
        # Disable the detection node.
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            if isinstance(mission, Missions.GpsArUcoSearch):
                self.supervisor.aruco.disable()
            elif isinstance(mission, Missions.GpsYoloSearch):
                self.supervisor.yolo.disable()

class SearchForArUco(Search):
    def __init__(self):
        super().__init__("search_for_aruco")

class SearchForYolo(Search):
    def __init__(self):
        super().__init__("search_for_yolo")
