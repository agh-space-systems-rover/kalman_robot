import time

from kalman_supervisor.state import State
from kalman_supervisor.modules import *

# Wait for a few seconds after lighting up the UEUOS LEDs.
# This is to warn the bystanders that Kalman will start moving autonomously.
WAIT_DURATION = 1


class Prepare(State):
    def __init__(self):
        super().__init__("prepare")

    def enter(self) -> None:
        self.supervisor.ueuos.set_state(Ueuos.State.AUTONOMY)
        self.entry_time = time.time()

    def tick(self) -> str | None:
        # Fall back to teleop if mission was ended early.
        if not self.supervisor.missions.has_mission():
            return "teleop"

        # Start moving after WAIT_DURATION seconds.
        if time.time() - self.entry_time > WAIT_DURATION:
            return "travel"
