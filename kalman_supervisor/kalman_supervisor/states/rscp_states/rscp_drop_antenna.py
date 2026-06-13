from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from kalman_interfaces.msg import ArcRscpRequest
import rclpy

from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from kalman_interfaces.msg import ArcRscpRequest
import rclpy

class RscpIdle(State):
    def __init__(self):
        super().__init__("rscp_drop_antenna")
        self.start_time = None

    def enter(self) -> None:
        self.supervisor.arc.drop_antenna()
        self.supervisor.get_logger().info(
            "[RSCP] Dropping antenna..."
        )
        self.start_time = self.supervisor.get_clock().now()

    def tick(self) -> str | None:
        if self.start_time is None:
            return "rscp_idle"

        elapsed_time = self.supervisor.get_clock().now() - self.start_time
        if elapsed_time.seconds_nanoseconds[0] >= 5:
            self.supervisor.get_logger().info("[RSCP Antenna] Task finished, returning to idle...")
            
            self.supervisor.rscp.send_task_finished()

            return "rscp_idle"
        
        return "rscp_idle"

    def exit(self) -> None:
        self.start_time = None
