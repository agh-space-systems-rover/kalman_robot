import time
from kalman_supervisor.state import State
from kalman_supervisor.modules import *

class RscpApproachAirlock(State):
    def __init__(self):
        super().__init__("rscp_approach_airlock")

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Entering approach airlock state")
        self.start_time = time.time()

    def tick(self) -> str | None:
        # Check if disarmed - abort and return to idle
        if not self.supervisor.rscp.is_armed():
            self.supervisor.get_logger().warn(
                "[RSCP] DISARM detected during airlock approach, returning to idle"
            )
            return "rscp_idle"
            
        if time.time() - self.start_time >= 2.0:
            self.supervisor.get_logger().info("[RSCP] Approach airlock completed (mock implementation), returning to idle")
            return "rscp_idle"
            
        return None

    def exit(self) -> None:
        pass
