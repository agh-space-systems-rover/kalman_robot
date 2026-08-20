#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from control_msgs.msg import JointJog
from kalman_interfaces.msg import ArmValues, ArmCompressed
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String, UInt16
import math
import xml.etree.ElementTree as ET
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
        self.declare_parameter("joint_limit_margins", [0.1] * 6)
        self.gripper_pos_open = self.get_parameter("gripper_pos_open").value
        self.gripper_pos_closed = self.get_parameter("gripper_pos_closed").value
        self.gripper_cmd_incr_per_deg = self.get_parameter(
            "gripper_cmd_incr_per_deg"
        ).value
        self.gripper_cmd_abs_open = self.get_parameter("gripper_cmd_abs_open").value
        self.gripper_cmd_abs_closed = self.get_parameter("gripper_cmd_abs_closed").value
        self.control_timeout = self.get_parameter("control_timeout").value
        self.control_rate = self.get_parameter("control_rate").value
        self.joint_limit_margins = list(
            self.get_parameter("joint_limit_margins").value
        )
        if len(self.joint_limit_margins) != 6:
            self.get_logger().warn(
                "joint_limit_margins must have 6 entries; using 0.1 rad"
            )
            self.joint_limit_margins = [0.1] * 6

        # State variables
        self.last_target_vel_joints = ArmValues()
        self.last_target_vel_joints_time = self.get_clock().now()
        self.last_target_vel_jaw = ArmValues()
        self.last_target_vel_jaw_time = self.get_clock().now()
        self.last_joint_state = JointState()
        self.last_gripper_pos = UInt16()
        self.joint_positions = {}
        self.joint_limits = {}
        self.joint_limits_ready = False
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
        robot_description_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.robot_description_sub = self.create_subscription(
            String,
            "/robot_description",
            self.robot_description_cb,
            robot_description_qos,
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
        self.joint_pos_pub = self.create_publisher(ArmValues, "new/current_pos", 10)

    def target_pos_jaw_cb(self, msg):
        if not self.joint_limits_ready or not math.isfinite(msg.jaw):
            return
        lower, upper = self.joint_limits["arm_joint_jaw"]
        target = min(max(msg.jaw, lower), upper)
        gripper_msg = UInt16()
        gripper_msg.data = int(
            lerp(
                self.gripper_cmd_abs_closed,
                self.gripper_cmd_abs_open,
                target / 1.57,
            )
        )
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

    def robot_description_cb(self, msg):
        try:
            root = ET.fromstring(msg.data)
            urdf_limits = {}
            for joint in root.findall("joint"):
                limit = joint.find("limit")
                if limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
                    urdf_limits[joint.attrib["name"]] = (
                        float(limit.attrib["lower"]),
                        float(limit.attrib["upper"]),
                    )

            names = [f"arm_joint_{i}" for i in range(1, 7)] + ["arm_joint_jaw"]
            effective_limits = {}
            for index, name in enumerate(names):
                lower, upper = urdf_limits[name]
                margin = self.joint_limit_margins[index] if index < 6 else 0.0
                lower += margin
                upper -= margin
                if lower > upper:
                    raise ValueError(f"limit margin leaves no range for {name}")
                effective_limits[name] = (lower, upper)
        except (ET.ParseError, KeyError, TypeError, ValueError) as error:
            self.joint_limits_ready = False
            self.get_logger().error(f"Failed to load arm joint limits: {error}")
            return

        self.joint_limits = effective_limits
        self.joint_limits_ready = True
        self.get_logger().info("Loaded joint limits from /robot_description")

    def limit_velocity(self, name, position, velocity):
        if not math.isfinite(position) or not math.isfinite(velocity):
            return 0.0
        lower, upper = self.joint_limits[name]
        if velocity < 0.0:
            if position <= lower:
                return 0.0
            return max(velocity, (lower - position) * self.control_rate)
        if velocity > 0.0:
            if position >= upper:
                return 0.0
            return min(velocity, (upper - position) * self.control_rate)
        return 0.0

    def control_timer_cb(self):
        now = self.get_clock().now()

        # Check if we should be commanding joints or gripper
        joints_active = (
            self.joint_limits_ready
            and len(self.joint_positions) == 6
            and (now - self.last_target_vel_joints_time).nanoseconds / 1e9
            < self.control_timeout
        )

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
                self.limit_velocity(
                    name,
                    self.joint_positions[name],
                    float(velocity),
                )
                for name, velocity in zip(
                    jog_msg.joint_names, self.last_target_vel_joints.joints
                )
            ]
            self.joint_jog_pub.publish(jog_msg)

        # Gripper control
        if (
            self.joint_limits_ready
            and (now - self.last_target_vel_jaw_time).nanoseconds / 1e9
            < self.control_timeout
        ):
            jaw_position = (
                float(self.gripper_pos_closed - self.last_gripper_pos.data)
                / (self.gripper_pos_closed - self.gripper_pos_open)
                * 1.57
            )
            jaw_velocity = self.limit_velocity(
                "arm_joint_jaw", jaw_position, self.last_target_vel_jaw.jaw
            )
            gripper_msg = Int8()
            gripper_msg.data = int(
                self.gripper_cmd_incr_per_deg
                * (jaw_velocity * 180 / math.pi)
                / self.control_rate
            )
            self.gripper_cmd_incr_pub.publish(gripper_msg)

    def joint_state_cb(self, msg):
        self.last_joint_state = msg
        names = [f"arm_joint_{i}" for i in range(1, 7)]
        if msg.name:
            positions_by_name = dict(zip(msg.name, msg.position))
            self.joint_positions = {
                name: positions_by_name[name]
                for name in names
                if name in positions_by_name
            }
        elif len(msg.position) >= 6:
            self.joint_positions = dict(zip(names, msg.position[:6]))
        else:
            self.joint_positions = {}
        self.pub_feedback()

    def gripper_pos_cb(self, msg):
        self.last_gripper_pos = msg
        self.pub_feedback()

    def pub_feedback(self):
        joint_msg = ArmValues()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = ""
        joint_msg.joints = np.zeros(6, dtype=np.float32)
        # Copy joint positions in arm joint order, independent of JointState order.
        for i in range(6):
            joint_msg.joints[i] = self.joint_positions.get(
                f"arm_joint_{i + 1}", 0.0
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
