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

        self.supervisor.get_logger().info(
            "[RSCP] Entered idle state, waiting for requests..."
        )

    def tick(self) -> str | None:
        goal = self.supervisor.rscp.get_navigation_goal()
        if goal is not None:
            stage = self.supervisor.rscp.get_current_stage()

            if stage is None or stage in [1, 2, 3]:
                self.supervisor.get_logger().warn(
                    f"[RSCP] NavigateToGPS received but unsupported stage {stage}"
                )
                # self.supervisor.rscp.clear_navigation_goal()
                # return None
                return "rscp_navigate_gps"
            elif stage == 4:
                self.supervisor.get_logger().info(
                    f"[RSCP] NavigateToGPS in stage 4, transitioning to rscp_navigate_gps"
                )
                return "rscp_navigate_gps"
            else:
                self.supervisor.get_logger().warn(
                    f"[RSCP] NavigateToGPS received but unknown stage {stage}"
                )
                self.supervisor.rscp.clear_navigation_goal()
                return None
            
        search_goal = self.supervisor.rscp.get_search_goal()
        if search_goal is not None:
            stage = self.supervisor.rscp.get_current_stage()
            if stage != 1:
                self.supervisor.get_logger().warn(
                    f"[RSCP] SearchGoal recieved but unsupported stage {stage}"
                )
            elif stage == 1 or stage == 2:
                self.supervisor.get_logger().info(
                    f"[RSCP] NavigateToGPS in stage 1 or 2, transitioning to rscp_search_goto_gps"
                )
                return "rscp_navigate_goto_gps"
                
        

            


        return None

    def exit(self) -> None:
        pass
