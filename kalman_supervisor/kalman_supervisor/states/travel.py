import numpy as np

from kalman_supervisor.state import State
from kalman_supervisor.modules import *


class Travel(State):
    def __init__(self):
        super().__init__("travel")

    def enter(self) -> None:
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            if isinstance(mission, Missions.TfGoal):
                self.supervisor.nav.send_goal(
                    np.array([mission.x, mission.y, 0]), mission.frame
                )
            elif isinstance(mission, Missions.GpsGoal):
                self.supervisor.nav.send_gps_goal(mission.lat, mission.lon)
            elif isinstance(mission, Missions.GpsArUcoSearch):
                self.supervisor.nav.send_gps_goal(mission.init_lat, mission.init_lon)
            elif isinstance(mission, Missions.GpsYoloSearch):
                self.supervisor.nav.send_gps_goal(mission.init_lat, mission.init_lon)

    def tick(self) -> str | None:
        # Cancel the navigation if missions was ended early.
        if not self.supervisor.missions.has_mission():
            self.supervisor.nav.cancel_goal()
            return "stop_to_teleop"

        # Go to finished or search if the goal was reached.
        if (
            self.supervisor.missions.has_mission()
            and not self.supervisor.nav.has_goal()
        ):
            mission = self.supervisor.missions.get_mission()
            if isinstance(mission, Missions.TfGoal):
                return "stop_to_finished"
            elif isinstance(mission, Missions.GpsGoal):
                return "stop_to_finished"
            elif isinstance(mission, Missions.GpsArUcoSearch):
                return "search_for_aruco"
            elif isinstance(mission, Missions.GpsYoloSearch):
                return "search_for_yolo"
