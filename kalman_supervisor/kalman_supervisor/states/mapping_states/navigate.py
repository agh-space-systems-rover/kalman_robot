import numpy as np

from kalman_supervisor.state import State, disable_state
from kalman_supervisor.modules import *

NAVIGATE_ROUGH_DISTANCE = 0.1  # 2.0


@disable_state
class Navigate(State):
    def __init__(self, name: str, early_exit_distance: float):
        super().__init__(name)
        self.early_exit_distance = early_exit_distance

    def enter(self) -> None:
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            assert isinstance(
                mission, Missions.MappingGoals
            ), "Mission is not of type MappingGoals"
            goal = mission.goals[mission.current_goal]
            assert goal.type in [
                Missions.MappingGoals.Goal.NAVIGATE_TO_PRECISE_LOCATION,
                Missions.MappingGoals.Goal.NAVIGATE_TO_ROUGH_LOCATION,
            ], "Current goal is not of type NAVIGATE_TO_*"
            pos_np = np.array([goal.location_x, goal.location_y, 0])
            self.supervisor.nav.send_goal(pos_np)

    def tick(self) -> str | None:
        # Cancel the navigation if missions was ended early.
        if not self.supervisor.missions.has_mission():
            self.supervisor.nav.cancel_goal()
            return "stop_to_teleop"

        # Early cancel the navigation if below tolerance threshold.
        if self.supervisor.nav.has_goal():
            if self.supervisor.nav.distance_to_goal() < self.early_exit_distance:
                self.supervisor.nav.cancel_goal()

        # Go to next goal if the position was reached.
        if (
            self.supervisor.missions.has_mission()
            and not self.supervisor.nav.has_goal()
        ):
            return "next_goal"


class NavigatePrecise(Navigate):
    def __init__(self):
        super().__init__("navigate_precise", 0.0)


class NavigateRough(Navigate):
    def __init__(self):
        super().__init__("navigate_rough", NAVIGATE_ROUGH_DISTANCE)
