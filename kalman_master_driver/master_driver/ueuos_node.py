# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import UInt8, UInt8MultiArray
# from ueuos.msg import ColorRGB
# from ueuos.srv import State, StateResponse, Color, ColorResponse


# UEUOS_STATE = {0: 'OFF',
#                1: 'AUTO',
#                2: 'MANUAL',
#                3: 'COMPLETED'}

# UEUOS_COLOR = {0: (0, 0, 0),
#                1: (255, 0, 0),
#                2: (0, 0, 255),
#                3: (0, 255, 0)}

# UEUOS_SET_STATE = 0x60
# UEUOS_SET_STATE_LEN = 1

# UEUOS_SET_COLOR = 0x61
# UEUOS_SET_COLOR_LEN = 3


# class LightsManager:
#     def __init__(self):
#         self.state = 2
#         self.rgb = UEUOS_COLOR[self.state]

#         self.set_state = rospy.Service('/ueuos/set_state', State, self.set_state_callback)
#         self.state_publisher = rospy.Publisher('/ueuos/state', UInt8, queue_size=10)
#         self.set_color = rospy.Service('/ueuos/set_color', Color, self.set_color_callback)
#         self.color_publisher = rospy.Publisher('/ueuos/color', ColorRGB, queue_size=10)
#         self.uart_publisher = rospy.Publisher('/kalman_rover/ros2uart', UInt8MultiArray, queue_size=10)

#     def publish(self):
#         self.state_publisher.publish(UInt8(self.state))
#         self.color_publisher.publish(ColorRGB(*self.rgb))

#     def set_state_callback(self, request):
#         if request.state in (0, 1, 2, 3):
#             self.state = request.state
#             self.rgb = UEUOS_COLOR[self.state]
#             self.uart_publisher.publish(UInt8MultiArray(data=[UEUOS_SET_STATE, UEUOS_SET_STATE_LEN, self.state]))
#             return StateResponse(True, f'Stan ustawiony na: {self.state} ({UEUOS_STATE[self.state]})')
#         else:
#             return StateResponse(False, f'Podaj stan: {UEUOS_STATE}')

#     def set_color_callback(self, request):
#         if all(0 <= color <= 255 for color in (request.red, request.green, request.blue)):
#             self.state = 0
#             self.rgb = (request.red, request.green, request.blue)
#             self.uart_publisher.publish(UInt8MultiArray(data=[UEUOS_SET_COLOR, UEUOS_SET_COLOR_LEN, *self.rgb]))
#             return ColorResponse(True, f'Kolor ustawiony na: {self.rgb}')
#         else:
#             return ColorResponse(False, f'Podaj kolor RGB')


# if __name__ == '__main__':
#     lm = LightsManager()

import rclpy
from rclpy.node import Node
from kalman_interfaces.srv import SetUeuosColor, SetUeuosEffect, SetUeuosMode
from std_msgs.msg import UInt8MultiArray

# Example service def:
# std_msgs/ColorRGBA color
# ---
# # no response

CAN_CMD_UEUOS_SET_STATE = 0x60
CAN_CMD_UEUOS_SET_COLOR = 0x61
CAN_CMD_UEUOS_SET_EFFECT = 0x62


class UeuosNode(Node):
    def __init__(self):
        super().__init__("ueuos_node")

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
            UInt8MultiArray(data=[CAN_CMD_UEUOS_SET_EFFECT, 1, request.effect.value])
        )
        return response

    def set_ueuos_mode(
        self, request: SetUeuosMode.Request, response: SetUeuosMode.Response
    ):
        self.ueuos_pub.publish(
            UInt8MultiArray(data=[CAN_CMD_UEUOS_SET_STATE, 1, request.mode.value])
        )
        return response


def main():
    rclpy.init()
    node = UeuosNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
