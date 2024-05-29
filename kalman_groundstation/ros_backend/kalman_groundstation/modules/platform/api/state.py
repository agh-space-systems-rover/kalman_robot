import rclpy
from fastapi import APIRouter

from rclpy.node import Node
from ..model.motors import PlatformState, WheelStates

from kalman_interfaces.msg import WheelStates as WheelStatesMsg, PlatformState as PlatformStateMsg

state_router = APIRouter(prefix="/state", tags=["state"])

STATE_PATH = "/station/system/rover/platform/state/"


# @state_router.get(
#     "/",
#     name="Get platform state",
#     response_model=PlatformState,
#     response_description="Returns current state of platform",
# )
# async def get():
#     # state = rclpy.get_param(STATE_PATH)
#     # TODO: Implement
#     return PlatformState()

class PlatformStateRouter(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="/state", tags=["state"])

        # Webserver stuff
        self.add_api_route(
            "/",
            self.get, # Callable
            name="Get platform state",
            response_model=PlatformState,
            response_description="Returns current state of platform",
            methods=["GET"],
        )

        self.platform_state = PlatformState()

        self.parent_node = parent_node

        self.motor_state_subscription = parent_node.create_subscription(
            WheelStatesMsg, "/wheel_states/return", self.motor_state_callback, qos_profile=10
        )

        self.motor_target_subscription = parent_node.create_subscription(
            WheelStatesMsg, "/wheel_states", self.motor_target_callback, qos_profile=10
        )

        self.platform_state_publisher = parent_node.create_publisher(
            PlatformStateMsg, "/station/wheels/state", qos_profile=10
        )
    
    def motor_state_callback(self, msg: WheelStatesMsg):
        self.platform_state.motors = WheelStates.from_ros_msg(msg)


    def motor_target_callback(self, msg: WheelStatesMsg):
        self.platform_state.target_motors = WheelStates.from_ros_msg(msg)

        self.platform_state_publisher.publish(self.platform_state.to_ros_msg())

    def get(self) -> PlatformState:
        return self.platform_state
