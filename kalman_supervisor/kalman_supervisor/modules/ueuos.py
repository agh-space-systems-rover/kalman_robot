from enum import IntEnum
from rclpy import Future

from kalman_supervisor.module import Module
from kalman_interfaces.srv import SetUeuosState, SetUeuosColor
from std_msgs.msg import ColorRGBA


class Ueuos(Module):
    class State(IntEnum):
        OFF = SetUeuosState.Request.OFF
        AUTONOMY = SetUeuosState.Request.AUTONOMY
        TELEOP = SetUeuosState.Request.TELEOP
        FINISHED = SetUeuosState.Request.FINISHED
    
    class RscpState(IntEnum):
        DISARMED = 0
        ARMED = 1
        AUTONOMOUS = 2

    def __init__(self):
        super().__init__("ueuos")

    def activate(self) -> None:
        self.__state_client = self.supervisor.create_client(SetUeuosState, "ueuos/set_state")
        self.__color_client = self.supervisor.create_client(SetUeuosColor, "ueuos/set_color")

        # Set a dummy state to avoid None in get_state()
        # It will be instantly overwritten when entering the teleop state.
        self.__state = Ueuos.State.OFF
        self.__wanted_state = Ueuos.State.OFF
        self.__req_future: Future = None
        self.__req_state: Ueuos.State | None = None
        
        # Track current color
        self.__current_color: ColorRGBA | None = None
        self.__wanted_color: ColorRGBA | None = None
        self.__color_req_future: Future = None
        self.__color_req: ColorRGBA | None = None

    def tick(self) -> None:
        # Handle standard state changes
        if self.__state != self.__wanted_state and self.__req_future is None:
            request = SetUeuosState.Request()
            request.state = self.__wanted_state

            if not self.__state_client.service_is_ready():
                self.supervisor.get_logger().info("Waiting for ueuos/set_state...")
                self.__state_client.wait_for_service()

            self.__req_future = self.__state_client.call_async(request)
            self.__req_state = self.__wanted_state

        # If a state request is pending and it is done, update the state and clear request.
        if self.__req_future is not None and self.__req_future.done():
            self.__state = self.__req_state
            self.__req_future = None
            self.__req_state = None
        
        # Handle RSCP color state changes
        if self.__wanted_color is not None and self.__color_req_future is None:
            # Check if the wanted color is different from current
            colors_differ = (
                self.__current_color is None or
                self.__current_color.r != self.__wanted_color.r or
                self.__current_color.g != self.__wanted_color.g or
                self.__current_color.b != self.__wanted_color.b
            )
            
            if colors_differ:
                request = SetUeuosColor.Request()
                request.color = self.__wanted_color
                
                if not self.__color_client.service_is_ready():
                    self.supervisor.get_logger().info("Waiting for ueuos/set_color...")
                    self.__color_client.wait_for_service()

                self.__color_req_future = self.__color_client.call_async(request)
                self.__color_req = self.__wanted_color
        
        # If an RSCP color request is pending and it is done, update and clear
        if self.__color_req_future is not None and self.__color_req_future.done():
            self.__current_color = self.__color_req
            self.__color_req_future = None
            self.__color_req = None

    def deactivate(self) -> None:
        self.supervisor.destroy_client(self.__state_client)
        self.supervisor.destroy_client(self.__color_client)

    def set_state(self, state: State) -> None:
        """Set standard UEUOS state (OFF/AUTONOMY/TELEOP/FINISHED)."""
        self.__wanted_state = state
    
    def set_rscp_state(self, rscp_state: RscpState) -> None:
        """Set RSCP-specific color state (DISARMED/ARMED/AUTONOMOUS)."""
        # Map RSCP states to colors
        if rscp_state == Ueuos.RscpState.DISARMED:
            self.__wanted_color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        elif rscp_state == Ueuos.RscpState.ARMED:
            self.__wanted_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        elif rscp_state == Ueuos.RscpState.AUTONOMOUS:
            self.__wanted_color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
