import rclpy
from rclpy.node import Node
import numpy as np
from struct import pack

from kalman_interfaces.msg import WheelStates, MasterMessage


METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR = 100


class WheelDriverNode(Node):
    def __init__(self):
        super().__init__("wheel_driver")
        self.create_subscription(
            WheelStates, "wheel_controller/state", self.controller_state_received, 1
        )
        self.publisher = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        self.autonomy_switch(True)

    def controller_state_received(self, msg: WheelStates):
        data = [
            msg.front_right.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.back_right.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.back_left.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.front_left.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            -np.rad2deg(msg.front_right.angle),
            np.rad2deg(msg.back_right.angle),
            np.rad2deg(msg.back_left.angle),
            -np.rad2deg(msg.front_left.angle),
        ]
        data = [int(x) for x in data]
        data = list(pack("b" * len(data), *data))
        self.publisher.publish(
            MasterMessage(cmd=MasterMessage.MOTOR_SET_WHEELS, data=data)
        )

    def autonomy_switch(self, on: bool):
        self.publisher.publish(
            MasterMessage(cmd=MasterMessage.AUTONOMY_SWITCH, data=[int(on) * 2])
        )


def main():
    rclpy.init()
    node = WheelDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
