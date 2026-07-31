import rclpy
import utm

from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from kalman_interfaces.msg import ArcRscpRequest
from std_msgs.msg import Empty


class RSCPDropAntenna(State):
    def __init__(self):
        super().__init__("rscp_drop_antenna")
        self.start_time = None

    def enter(self) -> None:
        self.supervisor.arc.drop_antenna(msg=Empty())
        start = self.supervisor.rscp.get_search_goal()
        robot_pos = self.supervisor.tf.robot_pos(frame="utm")

        zone_number = utm.latlon_to_zone_number(start[0], start[1])
        zone_letter = utm.latitude_to_zone_letter(start[0])
        lat, lon = utm.to_latlon(robot_pos[0], robot_pos[1], zone_number, zone_letter)
        self.supervisor.get_logger().info("[RSCP] Dropping antenna...")
        self.supervisor.rscp.send_gps_coordinate(lat, lon)
        self.start_time = self.supervisor.get_clock().now()

    def tick(self) -> str | None:
        if self.start_time is None:
            self.supervisor.rscp.clear_search_goal()
            return "rscp_idle"

        elapsed_time = self.supervisor.get_clock().now() - self.start_time
        if elapsed_time.nanoseconds >= 6 * 1e9:
            self.supervisor.get_logger().info(
                "[RSCP] Antenna Task finished, returning to idle..."
            )
            self.supervisor.rscp.send_task_finished()
            self.supervisor.rscp.clear_search_goal()
            return "rscp_idle"

    def exit(self) -> None:
        self.start_time = None
