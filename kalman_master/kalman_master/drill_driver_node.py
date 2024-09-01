import rclpy
from rclpy.node import Node
import numpy as np
from struct import pack
from kalman_interfaces.msg import Drill, MasterMessage

ARM_SPEED = 50
RACK_SPEED = 50
DRILL_SPEED = 100


class DrillDriver(Node):
    def __init__(self):
        super().__init__("drill_driver")

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        self.create_subscription(Drill, "drill", self.drill_data, 1)

    def drill_data(self, msg: Drill):
        msg.arm = np.clip(msg.arm, -1, 1)
        msg.rack = np.clip(msg.rack, -1, 1)
        msg.drill = np.clip(msg.drill, -1, 1)

        data_arm = [1 if msg.arm < 0 else 0, round(abs(msg.arm * ARM_SPEED))]
        data_rack = [1 if msg.rack < 0 else 0, round(abs(msg.rack * RACK_SPEED))]
        data_drill = [1 if msg.drill < 0 else 0, round(abs(msg.drill * DRILL_SPEED))]

        self.master_pub.publish(
            MasterMessage(cmd=MasterMessage.DRILL_ARM, data=data_arm)
        )
        self.master_pub.publish(
            MasterMessage(cmd=MasterMessage.DRILL_RACK, data=data_rack)
        )
        self.master_pub.publish(
            MasterMessage(cmd=MasterMessage.DRILL_DRILL, data=data_drill)
        )


def main():
    try:
        rclpy.init()
        node = DrillDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
