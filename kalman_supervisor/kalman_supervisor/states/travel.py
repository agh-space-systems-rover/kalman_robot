import time
import numpy as np

from kalman_supervisor.state import State
from kalman_supervisor.modules import *

# Stay in this state for additional STOP_DURATION seconds after stopping navigation.
STOP_DURATION = 2

class Travel(State):
    def __init__(self):
        super().__init__("travel")

    def enter(self) -> None:
        if self.supervisor.missions.has_mission():
            mission = self.supervisor.missions.get_mission()
            self.supervisor.nav.send_goal(np.array([mission.x, mission.y, 0]), mission.frame)
        self.travel_end_time: float | None = None
        self.mission_canceled = False

    def tick(self) -> str | None:
        # Start stopping timer if the goal was reached.
        if self.supervisor.missions.has_mission() and not self.travel_end_time and not self.supervisor.nav.has_goal():
            self.travel_end_time = time.time()

        # Cancel the navigation if missions was ended early.
        if not self.supervisor.missions.has_mission():
            self.supervisor.nav.cancel_goal()
            self.travel_end_time = time.time()
            # Set a flag to transition to teleop instead of finished.
            self.mission_canceled = True

        # Transition to finished or teleop once stopped.
        if self.travel_end_time is not None and time.time() - self.travel_end_time > STOP_DURATION:
            if self.mission_canceled:
                return "teleop"
            else:
                return "finished"
