import time

from kalman_supervisor.state import State, disable_state
from kalman_supervisor.modules import *

# Stay in this state for additional STOP_DURATION seconds after stopping navigation.
STOP_DURATION = 2


@disable_state
class Stop(State):
    def __init__(self, name: str):
        super().__init__(name)

    def enter(self) -> None:
        self.travel_end_time = time.time()

    def tick(self) -> str | None:
        # Transition to finished or teleop once stopped.
        if time.time() - self.travel_end_time > STOP_DURATION:
            if isinstance(self, StopToTeleop):
                return "teleop"
            elif isinstance(self, StopToFinished):
                return "finished"
            else:
                raise RuntimeError(
                    "This kind of stop state does not exist. It should never happen."
                )


class StopToTeleop(Stop):
    def __init__(self):
        super().__init__("stop_to_teleop")


class StopToFinished(Stop):
    def __init__(self):
        super().__init__("stop_to_finished")
