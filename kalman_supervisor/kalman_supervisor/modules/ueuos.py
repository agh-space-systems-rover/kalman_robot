from enum import IntEnum

from kalman_supervisor.module import Module
from kalman_interfaces.srv import SetUeuosState

class Ueuos(Module):
    class State(IntEnum):
        OFF = SetUeuosState.Request.OFF
        AUTONOMY = SetUeuosState.Request.AUTONOMY
        TELEOP = SetUeuosState.Request.TELEOP
        FINISHED = SetUeuosState.Request.FINISHED

    def __init__(self):
        super().__init__("ueuos")

    def activate(self) -> None:
        self.__client = self.supervisor.create_client(SetUeuosState, "ueuos/set_state")
        
        # Set a dummy state to avoid None in get_state()
        # It will be instantly overwritten when entering the teleop state.
        self.__state = Ueuos.State.OFF

    def deactivate(self) -> None:
        self.supervisor.destroy_client(self.__client)

    def get_state(self) -> State | None:
        return self.__state

    def set_state(self, state: State) -> None:
        request = SetUeuosState.Request()
        request.state = state

        if not self.__client.service_is_ready():
            self.supervisor.get_logger().info("Waiting for ueuos/set_state...")
            self.__client.wait_for_service()
        
        # TODO: The future should be recorded and synchronized so that subsequent service calls do not overlap.
        # This should not be a problem with sufficiently long waits between state changes.
        self.__client.call_async(request)
        self.__state = state
