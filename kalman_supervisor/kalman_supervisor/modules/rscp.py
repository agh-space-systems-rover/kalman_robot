from kalman_supervisor.module import Module
from kalman_interfaces.msg import ArcRscpRequest, ArcRscpResponse


class Rscp(Module):
    def __init__(self):
        super().__init__("rscp")

    def configure(self) -> None:
        self.supervisor.declare_parameter("rscp.enabled", False)

    def activate(self) -> None:
        self.module_enabled = self.supervisor.get_parameter("rscp.enabled").value
        if not self.module_enabled:
            return

        self.__armed = False
        self.__current_stage: int | None = None
        self.__pending_request: ArcRscpRequest | None = None
        self.__navigation_goal: tuple[float, float] | None = None  # (lat, lon)
        
        # Subscribe to rscp/req
        self.__req_sub = self.supervisor.create_subscription(
            ArcRscpRequest, 
            "rscp/req", 
            self.__req_callback, 
            10
        )
        
        # Publisher for rscp/res
        self.__res_pub = self.supervisor.create_publisher(
            ArcRscpResponse, 
            "rscp/res", 
            10
        )

    def __req_callback(self, msg: ArcRscpRequest) -> None:
        # Store the request for processing
        self.__pending_request = msg
        self.supervisor.get_logger().info(
            f"[RSCP] Received request type={msg.type} "
            f"(arm={msg.arm}, stage={msg.stage}, lat={msg.latitude}, lon={msg.longitude})"
        )
    
    def tick(self) -> None:
        if not self.module_enabled:
            return
        
        # Process certain request types in the module before states see them
        if self.__pending_request is not None:
            req = self.__pending_request
            
            # Handle ARM_DISARM requests immediately
            if req.type == ArcRscpRequest.ARM_DISARM:
                self.__armed = req.arm
                self.send_ack()
                self.__pending_request = None  # Consume the request
                
                self.supervisor.get_logger().info(
                    f"[RSCP] ARM_DISARM: {'ARMED' if self.__armed else 'DISARMED'}"
                )
                
                # Update UEUOS state based on armed status
                if self.__armed:
                    self.supervisor.ueuos.set_rscp_state(self.supervisor.ueuos.RscpState.ARMED)
                else:
                    self.supervisor.ueuos.set_rscp_state(self.supervisor.ueuos.RscpState.DISARMED)
            
            # Handle SET_STAGE requests immediately
            elif req.type == ArcRscpRequest.SET_STAGE:
                self.__current_stage = req.stage
                self.send_ack()
                self.__pending_request = None  # Consume the request
                
                self.supervisor.get_logger().info(
                    f"[RSCP] SET_STAGE: stage={self.__current_stage}"
                )
            
            # Other requests (NAV_TO_GPS) are left for states to handle

    def deactivate(self) -> None:
        if not self.module_enabled:
            return
        
        self.supervisor.destroy_subscription(self.__req_sub)
        self.supervisor.destroy_publisher(self.__res_pub)

    def is_armed(self) -> bool:
        return self.__armed

    def has_pending_request(self) -> bool:
        return self.__pending_request is not None

    def pop_pending_request(self) -> ArcRscpRequest | None:
        req = self.__pending_request
        self.__pending_request = None
        return req

    def peek_pending_request(self) -> ArcRscpRequest | None:
        return self.__pending_request

    def send_ack(self) -> None:
        msg = ArcRscpResponse()
        msg.type = ArcRscpResponse.ACK
        self.__res_pub.publish(msg)
        self.supervisor.get_logger().info("[RSCP] Sent ACK")

    def send_task_finished(self) -> None:
        msg = ArcRscpResponse()
        msg.type = ArcRscpResponse.TASK_FINISHED
        self.__res_pub.publish(msg)
        self.supervisor.get_logger().info("[RSCP] Sent TASK_FINISHED")

    def get_current_stage(self) -> int | None:
        return self.__current_stage
    
    def get_navigation_goal(self) -> tuple[float, float] | None:
        return self.__navigation_goal
    
    def set_navigation_goal(self, lat: float, lon: float) -> None:
        self.__navigation_goal = (lat, lon)
        self.supervisor.get_logger().info(f"[RSCP] Navigation goal set to ({lat}, {lon})")
    
    def clear_navigation_goal(self) -> None:
        self.__navigation_goal = None
        self.supervisor.get_logger().info("[RSCP] Navigation goal cleared")

