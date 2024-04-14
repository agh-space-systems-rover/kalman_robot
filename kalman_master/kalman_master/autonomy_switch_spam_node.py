import rclpy
from rclpy.node import Node

from kalman_interfaces.msg import MasterMessage

RATE = 1


class AutonomySwitchSpam(Node):
    def __init__(self):
        super().__init__("autonomy_switch_spam")
        self.pub = self.create_publisher(MasterMessage, "master_com/ros_to_master", 10)

        self.timer = self.create_timer(1 / RATE, self.timer_callback)

    def timer_callback(self):
        self.pub.publish(MasterMessage(cmd=MasterMessage.AUTONOMY_SWITCH, data=[1]))


def main():
    try:
        rclpy.init()
        node = AutonomySwitchSpam()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
