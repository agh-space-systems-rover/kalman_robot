import rclpy
from rclpy.node import Node
from kalman_interfaces.srv import SetUeuosColor, SetUeuosEffect, SetUeuosMode
from std_msgs.msg import UInt8MultiArray

CAN_CMD_UEUOS_SET_STATE = 0x60
CAN_CMD_UEUOS_SET_COLOR = 0x61
CAN_CMD_UEUOS_SET_EFFECT = 0x62


class UeuosNode(Node):
    def __init__(self):
        super().__init__("ueuos")

        # Init master publisher.
        self.ueuos_pub = self.create_publisher(
            UInt8MultiArray, "master_com/ros_to_master", 10
        )

        # Init services.
        self.create_service(SetUeuosColor, "set_ueuos_color", self.set_ueuos_color)
        self.create_service(SetUeuosEffect, "set_ueuos_effect", self.set_ueuos_effect)
        self.create_service(SetUeuosMode, "set_ueuos_mode", self.set_ueuos_mode)

    def set_ueuos_color(
        self, request: SetUeuosColor.Request, response: SetUeuosColor.Response
    ):
        self.ueuos_pub.publish(
            UInt8MultiArray(
                data=[
                    CAN_CMD_UEUOS_SET_COLOR,
                    3,
                    int(request.color.r * 255),
                    int(request.color.g * 255),
                    int(request.color.b * 255),
                ]
            )
        )
        return response

    def set_ueuos_effect(
        self, request: SetUeuosEffect.Request, response: SetUeuosEffect.Response
    ):
        self.ueuos_pub.publish(
            UInt8MultiArray(data=[CAN_CMD_UEUOS_SET_EFFECT, 1, request.effect])
        )
        return response

    def set_ueuos_mode(
        self, request: SetUeuosMode.Request, response: SetUeuosMode.Response
    ):
        self.ueuos_pub.publish(
            UInt8MultiArray(data=[CAN_CMD_UEUOS_SET_STATE, 1, request.mode])
        )
        return response


def main():
    rclpy.init()
    node = UeuosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
