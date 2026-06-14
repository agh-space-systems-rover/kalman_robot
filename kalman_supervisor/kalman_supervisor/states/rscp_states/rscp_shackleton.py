from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from kalman_interfaces.msg import ArcRscpRequest
import rclpy
import utm

from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from kalman_interfaces.msg import ArcRscpRequest
import rclpy


class RSCPShackleton(State):
    def __init__(self):
        super().__init__("rscp_shackleton")
        self.start_time = None

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Looking for the darkest boulder..")
        boulder_pos = self.supervisor.arc.get_boulder_position()
        last_goal = self.supervisor.rscp.get_search_goal()

        if last_goal is None:
            self.supervisor.get_logger().error(
                "[RSCP] No navigation goal set! Returning to rscp_idle"
            )
            self.failed = True
            return

        zone_number = utm.latlon_to_zone_number(last_goal.init_lat, last_goal.init_lon)
        zone_letter = utm.latitude_to_zone_letter(last_goal.init_lat)
        lat, lon = utm.to_latlon(
            boulder_pos[0], boulder_pos[1], zone_number, zone_letter
        )
        self.supervisor.rscp.send_gps_coordinate(latitude=lat, longitude=lon)
        self.supervisor.rscp.send_task_finished()

    def tick(self) -> str | None:
        if self.failed:
            self.supervisor.get_logger().error(
                "[RSCP] No navigation goal set! Returning to rscp_idle"
            )
            return "rscp_idle"
        else:
            self.supervisor.get_logger().info(
                "[RSCP] Sending boulder position, transitioning to idle"
            )
            return "rscp_idle"

    def exit(self) -> None:
        self.start_time = None
