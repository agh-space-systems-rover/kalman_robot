from kalman_supervisor.state import State
from kalman_supervisor.modules import *
from kalman_interfaces.msg import ArcRscpRequest


class RscpIdle(State):
    def __init__(self):
        super().__init__("rscp_idle")

    def enter(self) -> None:
        # Set UEUOS to show armed/disarmed state
        if self.supervisor.rscp.is_armed():
            self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.ARMED)
        else:
            self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.DISARMED)
        
        self.supervisor.get_logger().info("[RSCP] Entered idle state, waiting for requests...")

    def tick(self) -> str | None:
        # Check for pending requests (ARM_DISARM and SET_STAGE are handled by module)
        if not self.supervisor.rscp.has_pending_request():
            return None
        
        # Get the pending request
        req = self.supervisor.rscp.pop_pending_request()
        
        if req.type == ArcRscpRequest.NAV_TO_GPS:
            # Check if we're armed before allowing navigation
            if not self.supervisor.rscp.is_armed():
                self.supervisor.get_logger().warn(
                    "[RSCP] NavigateToGPS rejected: rover is DISARMED"
                )
                # Could send a NACK here if we had such a message type
                # For now, just stay in idle
                return None
            
            # Handle NavigateToGPS request
            # Store the goal in the module for the navigate state to use
            self.supervisor.rscp.set_navigation_goal(req.latitude, req.longitude)
            self.supervisor.rscp.send_ack()
            self.supervisor.get_logger().info(
                f"[RSCP] NavigateToGPS received (lat={req.latitude}, lon={req.longitude}), "
                f"sent ACK, transitioning to rscp_navigate_gps"
            )
            # Transition to navigate state
            return "rscp_navigate_gps"
            
        else:
            self.supervisor.get_logger().warn(
                f"[RSCP] Unknown request type {req.type}, ignoring"
            )
            return None

    def exit(self) -> None:
        pass
