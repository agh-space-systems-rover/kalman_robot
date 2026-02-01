import rclpy
from rclpy.node import Node
import numpy as np
from struct import pack
from kalman_interfaces.msg import Drill, MasterMessage

RACK_SPEED = 50
DRILL_SPEED = 100
MAX_ZEROFRAMES_SPAM = 5


class DrillDriver(Node):
    def __init__(self):
        super().__init__("drill_driver")
        print("Drill driver node started")
        self.zero_rack = 0
        self.zero_drill = 0

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        self.create_subscription(Drill, "drill", self.drill_data, 1)

    def drill_data(self, msg: Drill):
        msg.rack = np.clip(msg.rack, -1, 1)
        msg.drill = np.clip(msg.drill, 0, 1)

        data_rack = [0, 0, round(abs(msg.rack * RACK_SPEED)), 1 if msg.rack < 0 else 0]
        data_drill = [0, 0, round(abs(msg.drill * DRILL_SPEED))]
        self.zero_rack = self.zero_rack + 1 if self.is_zero_frame(data_rack) else 0
        self.zero_drill = self.zero_drill + 1 if self.is_zero_frame(data_drill) else 0

        # self.get_logger().info(f"Drill_zero {self.zero_rack}, Drill_zero {self.zero_drill}")
        if self.zero_rack <= MAX_ZEROFRAMES_SPAM:
            # self.get_logger().info(f"Publishing drill data: {data_drill}")
            self.master_pub.publish(
                MasterMessage(cmd=MasterMessage.DRILL_RACK, data=data_rack)
            )

        if self.zero_drill <= MAX_ZEROFRAMES_SPAM:
            # self.get_logger().info(f"Publishing drill data: {data_drill}")
            self.master_pub.publish(
                MasterMessage(cmd=MasterMessage.DRILL_DRILL, data=data_drill)
            )

    def is_zero_frame(self, data):
        return True if data[2] == 0 else False


def main():
    try:
        rclpy.init()
        node = DrillDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
