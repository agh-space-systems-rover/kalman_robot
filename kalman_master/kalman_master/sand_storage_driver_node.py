import rclpy
from rclpy.node import Node
import numpy as np
from kalman_interfaces.msg import MasterMessage
from std_msgs.msg import Float32, Empty

VALUE_LIMIT = 50

class SandStorageDriver(Node):
    def __init__(self):
        super().__init__("sand_storage_driver")

        self.master_pub = self.create_publisher(MasterMessage, "master_com/ros_to_master", 10)

        self.cmd_sub = self.create_subscription(
            Float32, "science/front_sand_storage/cmd_open_close", self.cmd_open_close_cb, 10
        )

        self.drop_antena_sub = self.create_subscription(
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

    def cmd_open_close_cb(self, msg: Float32):
        # 0x53 SAND_STORAGE_CMD
        # 0 - id universal
        # 0 - Id hbridge
        # 0-100 prędkość
        # 0 lub 1 kierunek

        val = np.clip(msg.data, -VALUE_LIMIT, VALUE_LIMIT)

        if val < 0:
            direction = 0
            speed = round(abs(val))
        else:
            direction = 1
            speed = round(val)

        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.SAND_STORAGE_CMD
        out_msg.data = [0, 0, speed, direction]

        self.master_pub.publish(out_msg)

def main():
    try:
        rclpy.init()
        node = SandStorageDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
