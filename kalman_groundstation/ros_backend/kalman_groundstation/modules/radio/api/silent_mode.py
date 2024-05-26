from rclpy.node import Node
from fastapi import APIRouter
from std_msgs.msg import UInt8MultiArray

from kalman_interfaces.msg import MasterMessage


class SilentModeRouter(APIRouter):
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        self.ros2uart_publisher = parent_node.create_publisher(
            MasterMessage, "/master_com/ros_to_master", qos_profile=10
        )

        # Web stuffs
        super().__init__(prefix="/silent_mode_on_off", tags=["configuration"])

        self.add_api_route(
            "/silent_on",
            self.silent_on, # Callable
            name="Sets silent mode",
            response_model=bool,
            methods=["PUT"],
        )


    def silent_on(self, target_state: bool):
        msg = MasterMessage()
        msg.cmd = 181
        robot_should_respond = 0 if target_state else 1
        msg.data = [robot_should_respond]
        self.ros2uart_publisher.publish(msg)
        
        return True
