import rclpy
import time

from rclpy.node import Node

from kalman_interfaces.srv import SetUeuosColor, SetUeuosEffect, SetUeuosState
from kalman_interfaces.msg import MasterMessage


class UeuosDriver(Node):
    def __init__(self):
        super().__init__("ueuos_driver")

        # Init master publisher.
        self.ueuos_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        # Init services.
        self.create_service(SetUeuosColor, "ueuos/set_color", self.set_color)
        self.create_service(SetUeuosEffect, "ueuos/set_effect", self.set_effect)
        self.create_service(SetUeuosState, "ueuos/set_state", self.set_state)

        self.tick_msg = MasterMessage(
            cmd=MasterMessage.UEUOS_SET_STATE, data=[SetUeuosState.Request.OFF]
        )
        self.create_timer(1.0, self.tick)

    def set_color(
        self, request: SetUeuosColor.Request, response: SetUeuosColor.Response
    ):
        self.tick_msg = MasterMessage(
            cmd=MasterMessage.UEUOS_SET_COLOR,
            data=[
                int(request.color.r * 255),
                int(request.color.g * 255),
                int(request.color.b * 255),
            ],
        )
        return response

    def set_effect(
        self, request: SetUeuosEffect.Request, response: SetUeuosEffect.Response
    ):
        self.tick_msg = MasterMessage(
            cmd=MasterMessage.UEUOS_SET_EFFECT, data=[request.effect]
        )
        return response

    def set_state(
        self, request: SetUeuosState.Request, response: SetUeuosState.Response
    ):
        self.tick_msg = MasterMessage(
            cmd=MasterMessage.UEUOS_SET_STATE, data=[request.state]
        )
        return response

    def tick(self):
        self.ueuos_pub.publish(self.tick_msg)


def main():
    try:
        rclpy.init()
        node = UeuosDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
