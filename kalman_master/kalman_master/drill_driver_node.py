import rclpy
from rclpy.node import Node
import numpy as np
from struct import pack

from kalman_interfaces.msg import ( Drill, MasterMessage )


ARM_SPEED = 50
RACK_SPEED = 50
DRILL_SPEED = 100


class DrillDriver(Node):
    def __init__(self):
        super().__init__("drill_driver")

        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        self.create_subscription(
            Drill, "drill", self.drill_data, 1
        )


    def drill_data(self, msg: Drill):
        msg.arm = np.clip(msg.arm, -1, 1)
        msg.rack = np.clip(msg.rack, -1, 1)
        msg.drill = np.clip(msg.drill, -1, 1)

        dataArm = [
            1 if msg.arm < 0 else 0,
            round(abs(msg.arm*ARM_SPEED))
        ]

        dataRack = [
            1 if msg.rack < 0 else 0,
            round(abs(msg.rack*RACK_SPEED))
        ]

        dataDrill = [
            1 if msg.drill < 0 else 0,
            round(abs(msg.drill*DRILL_SPEED))
        ]

        self.master_pub.publish(
            MasterMessage(cmd=MasterMessage.DRILL_ARM, data=dataArm)
        )
        self.master_pub.publish(
            MasterMessage(cmd=MasterMessage.DRILL_RACK, data=dataRack)
        )
        self.master_pub.publish(
            MasterMessage(cmd=MasterMessage.DRILL_DRILL, data=dataDrill)
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
