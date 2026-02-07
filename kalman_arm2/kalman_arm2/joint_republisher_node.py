#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from kalman_interfaces.msg import ArmValues


class JointRepublisher(Node):
    def __init__(self):
        super().__init__("joint_republisher")

        # Create publisher for joint states
        self.pub = self.create_publisher(JointState, "joint_states", 10)

        # Subscribe to joint status
        self.sub = self.create_subscription(
            ArmValues, "current_pos", self.on_arm_values, 10
        )

    def on_arm_values(self, msg):
        joint_state = JointState()
        joint_state.header = msg.header

        # Joint names for 6-DOF arm + jaw
        joint_state.name = [
            "arm_joint_1",
            "arm_joint_2",
            "arm_joint_3",
            "arm_joint_4",
            "arm_joint_5",
            "arm_joint_6",
            "arm_joint_jaw",
        ]

        # Copy joint positions
        joint_state.position = [0.0] * 7
        for i in range(6):
            joint_state.position[i] = msg.joints[i]
        joint_state.position[6] = msg.jaw

        self.pub.publish(joint_state)


def main():
    try:
        rclpy.init()
        node = JointRepublisher()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
