import time

import numpy as np

from kalman_supervisor.state import State
from kalman_supervisor.modules import *

WAIT_ON_ENTRY = 1.0


class LoopClosure(State):
    def __init__(self):
        super().__init__("loop_closure")

    def enter(self) -> None:
        self.entry_time = time.time()
        self.started_rotation = False

    def tick(self) -> str | None:
        # Cancel the navigation if missions was ended early.
        if not self.supervisor.missions.has_mission():
            self.supervisor.cmd_vel.cancel_rotation_in_place()
            return "stop_to_teleop"

        # Wait for a few seconds before starting the rotation.
        if time.time() - self.entry_time < WAIT_ON_ENTRY:
            return

        # Rotate in place.
        if (
            not self.supervisor.cmd_vel.is_rotating_in_place()
            and not self.started_rotation
        ):
            mission = self.supervisor.missions.get_mission()
            assert isinstance(
                mission, Missions.MappingGoals
            ), "Mission is not of type MappingGoals"
            goal = mission.goals[mission.current_goal]
            assert (
                goal.type == Missions.MappingGoals.Goal.ATTEMPT_LOOP_CLOSURE
            ), "Current goal is not of type ATTEMPT_LOOP_CLOSURE"
            self.supervisor.cmd_vel.rotate_in_place(angular_z=1.0, duration=10.0)
            self.started_rotation = True

        # Once the rotation is complete, go to the next goal.
        if (
            self.supervisor.missions.has_mission()
            and not self.supervisor.cmd_vel.is_rotating_in_place()
            and self.started_rotation
        ):
            return "next_goal"
