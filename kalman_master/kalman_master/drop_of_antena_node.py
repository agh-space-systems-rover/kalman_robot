import rclpy
from rclpy.node import Node
import numpy as np
from kalman_interfaces.msg import MasterMessage
from std_msgs.msg import Empty

VALUE_LIMIT = 50

class DropOfAnenaDriver(Node):
    def __init__(self):
        super().__init__("drop_of_antena_driver")

        self.master_pub = self.create_publisher(MasterMessage, "master_com/ros_to_master", 10)

        self.cmd_sub = self.create_subscription(
            Empty, "science/front_sand_storage/drop_of_antenna", self.drop_of_antenna, 10
        )

    def drop_of_antenna(self, msg: Empty):
        # 0x5E GS_TO_UNIVERSAL
        # 0 - id universal
        # 0 - Id hbridge

        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_UNIVERSAL
        out_msg.data = [0, 0]

        self.master_pub.publish(out_msg)

def main():
    try:
        rclpy.init()
        node = DropOfAnenaDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
