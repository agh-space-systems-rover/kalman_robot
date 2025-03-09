import numpy as np
import time

from kalman_supervisor.state import State
from kalman_supervisor.modules import *


class TakePhotos(State):
    def __init__(self):
        super().__init__("take_photos")
        self._step = 0
        self._last_action_time = 0

    def enter(self) -> None:
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            assert isinstance(
                mission, Missions.MappingGoals
            ), "Mission is not of type MappingGoals"
            goal = mission.goals[mission.current_goal]
            assert (
                goal.type == Missions.MappingGoals.Goal.TAKE_PHOTOS
            ), "Current goal is not of type TAKE_PHOTOS"
            self._step = 0
            self._last_action_time = time.time()

    def tick(self) -> str | None:
        if not self.supervisor.missions.has_mission():
            return "stop_to_teleop"

        mission = self.supervisor.missions.get_mission()

        current_time = time.time()

        if self._step == 0 and current_time - self._last_action_time >= 1.0:
            self.supervisor.arch.take_photos(mission.next_photo_label)
            mission.next_photo_label += 1
            self._step = 1
            self._last_action_time = current_time
        elif self._step == 1 and current_time - self._last_action_time >= 1.0:
            self.supervisor.cmd_vel.rotate_in_place(
                angular_z=0.5,
                duration=3.0,
            )  # ~45 degrees
            self._step = 2
            self._last_action_time = current_time
        elif self._step == 2 and not self.supervisor.cmd_vel.is_rotating_in_place():
            self.supervisor.arch.take_photos(mission.next_photo_label)
            mission.next_photo_label += 1
            self._step = 3
            self._last_action_time = current_time
        elif self._step == 3 and current_time - self._last_action_time >= 1.0:
            return "next_goal"
