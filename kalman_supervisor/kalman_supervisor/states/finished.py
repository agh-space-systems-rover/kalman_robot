import time

from kalman_supervisor.state import State
from kalman_supervisor.modules import *

# Go back to teleop after this many seconds.
TIMEOUT = 30

class Finished(State):
    def __init__(self):
        super().__init__("finished")

    def enter(self) -> None:
        self.supervisor.ueuos.set_state(Ueuos.State.FINISHED)
        self.supervisor.missions.succeed_mission()
        self.entry_time = time.time()

    def tick(self) -> str | None:
        # Go back to teleop after TIMEOUT seconds.
        if time.time() - self.entry_time > TIMEOUT:
            return "teleop"

        # Or if there's a new mission.
        # Teleop will wait for a few seconds before starting to prepare.
        if self.supervisor.missions.has_mission():
            return "teleop"
        