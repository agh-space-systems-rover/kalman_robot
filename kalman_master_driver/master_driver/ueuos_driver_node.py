import rclpy
from rclpy.node import Node

from kalman_interfaces.srv import SetUeuosColor, SetUeuosEffect, SetUeuosMode
from kalman_interfaces.msg import MasterMessage


class UeuosNode(Node):
    def __init__(self):
        super().__init__("ueuos_driver")

        # Init master publisher.
        self.ueuos_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        # Init services.
        self.create_service(SetUeuosColor, "ueuos/set_color", self.set_color)
        self.create_service(SetUeuosEffect, "ueuos/set_effect", self.set_effect)
        self.create_service(SetUeuosMode, "ueuos/set_mode", self.set_mode)

    def set_color(
        self, request: SetUeuosColor.Request, response: SetUeuosColor.Response
    ):
        self.ueuos_pub.publish(
            MasterMessage(
                cmd=MasterMessage.UEUOS_SET_COLOR,
                data=[
                    int(request.color.r * 255),
                    int(request.color.g * 255),
                    int(request.color.b * 255),
                ],
            )
        )
        return response

    def set_effect(
        self, request: SetUeuosEffect.Request, response: SetUeuosEffect.Response
    ):
        self.ueuos_pub.publish(
            MasterMessage(cmd=MasterMessage.UEUOS_SET_EFFECT, data=[request.effect])
        )
        return response

    def set_mode(self, request: SetUeuosMode.Request, response: SetUeuosMode.Response):
        self.ueuos_pub.publish(
            MasterMessage(cmd=MasterMessage.UEUOS_SET_STATE, data=[request.mode])
        )
        return response


def main():
    rclpy.init()
    node = UeuosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
