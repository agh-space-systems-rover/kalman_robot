import rclpy
from rclpy.node import Node
import numpy as np
from struct import pack

from kalman_interfaces.msg import WheelStates, MasterMessage

CMD_MOTOR_SET_WHEELS = 0x40
CMD_AUTONOMY_SWITCH = 0x20

METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR = 100


class WheelDriverNode(Node):
    def __init__(self):
        super().__init__("wheel_driver")
        self.create_subscription(
            WheelStates, "/wheel_controller/state", self.controller_state_received, 1
        )
        self.publisher = self.create_publisher(
            MasterMessage, "/master_com/ros_to_master", 10
        )

        self.autonomy_switch(True)

    def controller_state_received(self, msg: WheelStates):
        msg = MasterMessage()
        msg.command = MasterMessage.CMD_MOTOR_SET_WHEELS
        msg.data = [
            msg.front_right.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.back_right.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.back_left.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            msg.front_left.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR,
            -np.rad2deg(msg.front_right.angle),
            np.rad2deg(msg.back_right.angle),
            np.rad2deg(msg.back_left.angle),
            -np.rad2deg(msg.front_left.angle),
        ]
        msg.data = [int(x) for x in msg.data]
        msg.data = list(pack("b" * len(msg.data), *msg.data))
        self.publisher.publish(msg)

    def autonomy_switch(self, on: bool):
        msg = MasterMessage()
        msg.command = MasterMessage.CMD_AUTONOMY_SWITCH
        msg.data = [int(on) * 2]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = WheelDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
