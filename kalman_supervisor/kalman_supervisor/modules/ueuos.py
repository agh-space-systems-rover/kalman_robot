from enum import IntEnum
from rclpy import Future

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
        self.__wanted_state = Ueuos.State.OFF
        self.__req_future: Future = None
        self.__req_state: Ueuos.State | None = None

    def tick(self) -> None:
        # If state change is needed and request is not pending, send the request.
        if self.__state != self.__wanted_state and self.__req_future is None:
            request = SetUeuosState.Request()
            request.state = self.__wanted_state

            if not self.__client.service_is_ready():
                self.supervisor.get_logger().info("Waiting for ueuos/set_state...")
                self.__client.wait_for_service()

            self.__req_future = self.__client.call_async(request)
            self.__req_state = self.__wanted_state

        # If a request is pending and it is done, update the state and clear request.
        if self.__req_future is not None and self.__req_future.done():
            self.__state = self.__req_state
            self.__req_future = None
            self.__req_state = None

    def deactivate(self) -> None:
        self.supervisor.destroy_client(self.__client)

    def get_state(self) -> State:
        return self.__state

    def set_state(self, state: State) -> None:
        self.__wanted_state = state
