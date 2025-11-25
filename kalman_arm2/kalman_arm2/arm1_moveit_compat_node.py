#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from control_msgs.msg import JointJog
from kalman_interfaces.msg import ArmValues, ArmCompressed
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, UInt16
import math
import numpy as np

def lerp(a, b, t):
    return a + (b - a) * t

class Arm1MoveitCompatNode(Node):
    def __init__(self):
        super().__init__("arm1_moveit_compat")

        # Declare parameters
        self.declare_parameter("gripper_pos_open", 2100)
        self.declare_parameter("gripper_pos_closed", 3070)
        self.declare_parameter("gripper_cmd_incr_per_deg", -10)
        self.declare_parameter("gripper_cmd_abs_open", 1570)
        self.declare_parameter("gripper_cmd_abs_closed", 2400)
        self.declare_parameter("control_timeout", 0.1)
        self.declare_parameter("control_rate", 10.0)
        self.gripper_pos_open = self.get_parameter("gripper_pos_open").value
        self.gripper_pos_closed = self.get_parameter("gripper_pos_closed").value
        self.gripper_cmd_incr_per_deg = self.get_parameter("gripper_cmd_incr_per_deg").value
        self.gripper_cmd_abs_open = self.get_parameter("gripper_cmd_abs_open").value
        self.gripper_cmd_abs_closed = self.get_parameter("gripper_cmd_abs_closed").value
        self.control_timeout = self.get_parameter("control_timeout").value
        self.control_rate = self.get_parameter("control_rate").value

        # State variables
        self.last_target_vel_joints = ArmValues()
        self.last_target_vel_joints_time = self.get_clock().now()
        self.last_target_vel_jaw = ArmValues()
        self.last_target_vel_jaw_time = self.get_clock().now()
        self.last_joint_state = JointState()
        self.last_gripper_pos = UInt16()
        self.was_commanding = False  # Track if we were sending commands last cycle

        # Control publishers/subscribers
        self.target_pos_jaw_sub = self.create_subscription(
            ArmValues, "new/target_pos/jaw", self.target_pos_jaw_cb, 10
        )
        self.target_vel_sub = self.create_subscription(
            ArmValues, "new/target_vel", self.target_vel_cb, 10
        )
        self.target_vel_joints_sub = self.create_subscription(
            ArmValues, "new/target_vel/joints", self.target_vel_joints_cb, 10
        )
        self.target_vel_jaw_sub = self.create_subscription(
            ArmValues, "new/target_vel/jaw", self.target_vel_jaw_cb, 10
        )
        self.joint_jog_pub = self.create_publisher(
            JointJog, "old/servo_node/delta_joint_cmds", 10
        )
        self.gripper_cmd_incr_pub = self.create_publisher(
            Int8, "old/gripper/command_incremental", 10
        )
        self.gripper_cmd_abs_pub = self.create_publisher(
            UInt16, "old/gripper/command_absolute", 10
        )
        self.joy_compressed_pub = self.create_publisher(
            ArmCompressed, "old/joy_compressed", 10
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_rate, self.control_timer_cb
        )

        # Feedback publishers/subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, "old/arm_controllers/joint_states", self.joint_state_cb, 10
        )
        self.gripper_pos_sub = self.create_subscription(
            UInt16, "old/gripper/position", self.gripper_pos_cb, 10
        )
        self.joint_pos_pub = self.create_publisher(
            ArmValues, "new/current_pos", 10
        )

    def target_pos_jaw_cb(self, msg):
        gripper_msg = UInt16()
        gripper_msg.data = int(lerp(self.gripper_cmd_abs_closed, self.gripper_cmd_abs_open, msg.jaw / 1.57))
        self.gripper_cmd_abs_pub.publish(gripper_msg)

    def target_vel_cb(self, msg):
        self.last_target_vel_joints = msg
        self.last_target_vel_joints_time = self.get_clock().now()
        self.last_target_vel_jaw = msg
        self.last_target_vel_jaw_time = self.get_clock().now()

    def target_vel_joints_cb(self, msg):
        self.last_target_vel_joints = msg
        self.last_target_vel_joints_time = self.get_clock().now()

    def target_vel_jaw_cb(self, msg):
        self.last_target_vel_jaw = msg
        self.last_target_vel_jaw_time = self.get_clock().now()

    def control_timer_cb(self):
        now = self.get_clock().now()

        # Check if we should be commanding joints or gripper
        joints_active = (
            now - self.last_target_vel_joints_time
        ).nanoseconds / 1e9 < self.control_timeout

        # Send joy_compressed message to trigger servo mode when commands start
        if joints_active and not self.was_commanding:
            joy_msg = ArmCompressed()
            joy_msg.joints_mask = 0  # Empty mask
            joy_msg.joints_data = []
            self.joy_compressed_pub.publish(joy_msg)

        self.was_commanding = joints_active

        # 6-DoF
        if joints_active:
            jog_msg = JointJog()
            jog_msg.header.stamp = now.to_msg()
            jog_msg.joint_names = [
                "arm_joint_1",
                "arm_joint_2",
                "arm_joint_3",
                "arm_joint_4",
                "arm_joint_5",
                "arm_joint_6",
            ]
            jog_msg.velocities = [
                float(v) for v in self.last_target_vel_joints.joints
            ]
            self.joint_jog_pub.publish(jog_msg)

        # Gripper control
        if (
            now - self.last_target_vel_jaw_time
        ).nanoseconds / 1e9 < self.control_timeout:
            gripper_msg = Int8()
            gripper_msg.data = int(
                self.gripper_cmd_incr_per_deg
                * (self.last_target_vel_jaw.jaw * 180 / math.pi)
                / self.control_rate
            )
            self.gripper_cmd_incr_pub.publish(gripper_msg)

    def joint_state_cb(self, msg):
        self.last_joint_state = msg
        self.pub_feedback()

    def gripper_pos_cb(self, msg):
        self.last_gripper_pos = msg
        self.pub_feedback()

    def pub_feedback(self):
        joint_msg = ArmValues()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = ""
        joint_msg.joints = np.zeros(6, dtype=np.float32)
        # Copy joint positions
        for i in range(6):
            joint_msg.joints[i] = (
                self.last_joint_state.position[i]
                if len(self.last_joint_state.position) > i
                else 0.0
            )
        # Calculate jaw position
        joint_msg.jaw = (
            float(self.gripper_pos_closed - self.last_gripper_pos.data)
            / (self.gripper_pos_closed - self.gripper_pos_open)
            * 1.57
        )
        self.joint_pos_pub.publish(joint_msg)


def main():
    try:
        rclpy.init()
        node = Arm1MoveitCompatNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
