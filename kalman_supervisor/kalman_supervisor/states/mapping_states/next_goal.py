import time

from kalman_supervisor.state import State
from kalman_supervisor.modules import *


class NextGoal(State):
    def __init__(self):
        super().__init__("next_goal")

    def tick(self) -> str | None:
        # Fall back to teleop if mission was ended early.
        if not self.supervisor.missions.has_mission():
            return "stop_to_teleop"

        # Otherwise go to the next goal.
        mission: Missions.MappingGoals = self.supervisor.missions.get_mission()
        assert isinstance(
            mission, Missions.MappingGoals
        ), "Mission is not of type MappingGoals"
        mission.current_goal += 1
        if mission.current_goal >= len(mission.goals):
            return "stop_to_finished"
        else:
            type = mission.goals[mission.current_goal].type
            match type:
                case Missions.MappingGoals.Goal.NAVIGATE_TO_PRECISE_LOCATION:
                    return "navigate_precise"
                case Missions.MappingGoals.Goal.NAVIGATE_TO_ROUGH_LOCATION:
                    return "navigate_rough"
                case Missions.MappingGoals.Goal.ATTEMPT_LOOP_CLOSURE:
                    return "loop_closure"
                case Missions.MappingGoals.Goal.TAKE_PHOTOS:
                    return "take_photos"
                case _:
                    self.supervisor.get_logger().warn(
                        f"[NextGoal] Unknown goal type: {type}. Stopping the mission."
                    )
                    return "stop_to_finished"
