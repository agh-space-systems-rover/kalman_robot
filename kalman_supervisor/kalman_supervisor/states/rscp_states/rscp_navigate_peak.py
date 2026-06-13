from kalman_supervisor.state import State
from kalman_supervisor.modules import *
import utm

class RscpNavigatePeak(State):
    def __init__(self):
        super().__init__("rscp_navigate_peak")

    def enter(self) -> None:
        # Set UEUOS to autonomous (yellow)
        self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.AUTONOMOUS)

        # Get the navigation goal from the RSCP module
        start = self.supervisor.rscp.get_search_goal()
        self.supervisor.rscp.clear_search_goal()
        peak_pos = self.supervisor.arc.get_peak_position()

        zone_number = utm.latlon_to_zone_number(
                    start.init_lat, start.init_lon
                )
        zone_letter = utm.latitude_to_zone_letter(start.init_lat)
        lat, lon = utm.to_latlon(
                    peak_pos[0], peak_pos[1], zone_number, zone_letter
                )
        if peak_pos is None:
            self.supervisor.get_logger().error(
                "[RSCP] No peak position recieved!"
            )
            self.failed = True
            return

        self.supervisor.get_logger().info(
            f"[RSCP] Starting GPS navigation to ({lat}, {lon})"
        )

        # Send the GPS goal to navigation
        self.supervisor.nav.send_gps_goal(lat, lon)
        self.failed = False

    def tick(self) -> str | None:
        # If we failed to set up navigation, return to idle
        if self.failed:
            self.supervisor.rscp.clear_search_goal()
            return "rscp_idle"

        # Check if disarmed - abort and return to idle
        if not self.supervisor.rscp.is_armed():
            self.supervisor.get_logger().warn(
                "[RSCP] DISARM detected during navigation, aborting"
            )
            self.supervisor.rscp.clear_search_goal()
            return "rscp_idle"

        # Check if navigation is complete
        if not self.supervisor.nav.has_goal():
            self.supervisor.get_logger().info("[RSCP] NavigateToPeak navigation completed")
            # Send TASK_FINISHED response
            self.supervisor.rscp.send_task_finished()
            stage = self.supervisor.rscp.get_current_stage()
            if stage != 1:
                self.supervisor.get_logger().warn(
                f"[RSCP] Stage is not 1, current is {stage} while in NavigateToPeak "
            )
                
            self.supervisor.rscp.clear_search_goal()
            # Return to idle state to wait for next request
            return "rscp_drop_antenna"

        # Still navigating
        return None

    def exit(self) -> None:
        # Cancel navigation if we're exiting prematurely
        if self.supervisor.nav.has_goal():
            self.supervisor.get_logger().info(
                "[RSCP] Exiting rscp_navigate_gps, canceling navigation"
            )
            self.supervisor.nav.cancel_goal()
