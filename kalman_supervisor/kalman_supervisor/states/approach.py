from kalman_supervisor.state import State
from kalman_supervisor.modules import *


class Approach(State):
    def __init__(self):
        super().__init__("approach")

    def enter(self) -> None:
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            if isinstance(mission, Missions.GpsArUcoSearch):
                self.supervisor.nav.send_gps_goal(
                    mission.marker_lat, mission.marker_lon
                )
            elif isinstance(mission, Missions.GpsYoloSearch):
                self.supervisor.nav.send_gps_goal(mission.obj_lat, mission.obj_lon)

    def tick(self) -> str | None:
        # Cancel the navigation if missions was ended early.
        if not self.supervisor.missions.has_mission():
            self.supervisor.nav.cancel_goal()
            return "stop_to_teleop"

        # Go to finished if the goal was reached.
        if not self.supervisor.nav.has_goal():
            return "stop_to_finished"
