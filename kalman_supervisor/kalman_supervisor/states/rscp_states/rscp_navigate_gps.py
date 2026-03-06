from kalman_supervisor.state import State
from kalman_supervisor.modules import *


class RscpNavigateGps(State):
    def __init__(self):
        super().__init__("rscp_navigate_gps")

    def enter(self) -> None:
        # Set UEUOS to autonomous (yellow)
        self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.AUTONOMOUS)
        
        # Get the navigation goal from the RSCP module
        goal = self.supervisor.rscp.get_navigation_goal()
        
        if goal is None:
            self.supervisor.get_logger().error(
                "[RSCP] No navigation goal set! Returning to rscp_idle"
            )
            self.failed = True
            return
        
        lat, lon = goal
        self.supervisor.get_logger().info(
            f"[RSCP] Starting GPS navigation to ({lat}, {lon})"
        )
        
        # Send the GPS goal to navigation
        self.supervisor.nav.send_gps_goal(lat, lon)
        self.failed = False

    def tick(self) -> str | None:
        # If we failed to set up navigation, return to idle
        if self.failed:
            return "rscp_idle"
        
        # Check if disarmed - abort and return to idle
        if not self.supervisor.rscp.is_armed():
            self.supervisor.get_logger().warn(
                "[RSCP] DISARM detected during navigation, aborting"
            )
            # Navigation will be canceled in exit()
            return "rscp_idle"
        
        # Check if navigation is complete
        if not self.supervisor.nav.has_goal():
            self.supervisor.get_logger().info(
                "[RSCP] GPS navigation completed"
            )
            # Send TASK_FINISHED response
            self.supervisor.rscp.send_task_finished()
            # Clear the navigation goal
            self.supervisor.rscp.clear_navigation_goal()
            # Return to idle state to wait for next request
            return "rscp_idle"
        
        # Still navigating
        return None

    def exit(self) -> None:
        # Cancel navigation if we're exiting prematurely
        if self.supervisor.nav.has_goal():
            self.supervisor.get_logger().info(
                "[RSCP] Exiting rscp_navigate_gps, canceling navigation"
            )
            self.supervisor.nav.cancel_goal()
