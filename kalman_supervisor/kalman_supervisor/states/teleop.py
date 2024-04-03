import time

from kalman_supervisor.state import State
from kalman_supervisor.modules import *

# Minimum time for which to stay in this state.
# This is meant to warn bystanders that a mission is being uploaded by the operator.
MINIMUM_DURATION = 1


class Teleop(State):
    def __init__(self):
        super().__init__("teleop")

    def enter(self) -> None:
        self.supervisor.ueuos.set_state(Ueuos.State.TELEOP)
        self.entry_time = time.time()

    def tick(self) -> str | None:
        # Stay in this state for at least MINIMUM_DURATION seconds.
        if time.time() - self.entry_time < MINIMUM_DURATION:
            return

        # If there's a mission, start preparing.
        if self.supervisor.missions.has_mission():
            return "prepare"
