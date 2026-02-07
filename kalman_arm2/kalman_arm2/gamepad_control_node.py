#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
from kalman_interfaces.msg import ArmValues
import numpy as np


LEFT_X = 0
LEFT_Y = 1
LEFT_TRIGGER = 2
RIGHT_X = 3
RIGHT_Y = 4
RIGHT_TRIGGER = 5

SOUTH_BUTTON = 0
NORTH_BUTTON = 3
LEFT_SHOULDER = 4
RIGHT_SHOULDER = 5


class GamepadControl(Node):
    def __init__(self):
        super().__init__("gamepad_control")

        # Declare parameters
        self.declare_parameter("linear_scale", 0.1)  # m/s
        self.declare_parameter("angular_scale", 0.5)  # rad/s
        self.declare_parameter("roll_scale", 0.3)  # rad/s
        self.declare_parameter("jaw_vel_scale", 1.0)  # rad/s
        self.declare_parameter("end_effector_frame", "arm_link_end")

        self.linear_scale = self.get_parameter("linear_scale").value
        self.angular_scale = self.get_parameter("angular_scale").value
        self.roll_scale = self.get_parameter("roll_scale").value
        self.jaw_vel_scale = self.get_parameter("jaw_vel_scale").value
        self.end_effector_frame = self.get_parameter("end_effector_frame").value

        self.last_twist_msg = TwistStamped()

        # Publishers & subscribers
        self.joy_sub = self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.twist_pub = self.create_publisher(TwistStamped, "target_twist", 10)
        self.jaw_pub = self.create_publisher(ArmValues, "jaw_vel", 10)

        self.get_logger().info("Gamepad arm control node started")

    def on_joy(self, msg):
        # Check for sufficient axes/buttons
        if len(msg.axes) < 6 or len(msg.buttons) < 6:
            self.get_logger().warn("Insufficient gamepad axes or buttons")
            return

        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.end_effector_frame

        # Left stick: controls linear in Y/Z plane
        twist_msg.twist.linear.y = msg.axes[LEFT_X] * self.linear_scale
        twist_msg.twist.linear.z = msg.axes[LEFT_Y] * self.linear_scale

        # Right stick: controls pitch and yaw
        twist_msg.twist.angular.y = -msg.axes[RIGHT_Y] * self.angular_scale  # pitch
        twist_msg.twist.angular.z = msg.axes[RIGHT_X] * self.angular_scale  # yaw

        # Triggers control linear fwd/bwd (X)
        left_trigger_val = (
            -msg.axes[LEFT_TRIGGER] * 0.5 + 0.5
        )  # Convert from [-1,1] to [0,1]
        right_trigger_val = (
            -msg.axes[RIGHT_TRIGGER] * 0.5 + 0.5
        )  # Convert from [-1,1] to [0,1]
        twist_msg.twist.linear.x = (
            right_trigger_val - left_trigger_val
        ) * self.linear_scale

        # Shoulders: control roll (digital input)
        roll_input = 0.0
        if msg.buttons[LEFT_SHOULDER]:
            roll_input -= 1.0
        if msg.buttons[RIGHT_SHOULDER]:
            roll_input += 1.0
        twist_msg.twist.angular.x = roll_input * self.roll_scale

        # Only publish if message has changed or is non-zero
        if not self.twist_msg_is_zero(twist_msg) or not self.twist_msg_is_zero(
            self.last_twist_msg
        ):
            self.twist_pub.publish(twist_msg)
            self.last_twist_msg = twist_msg

        # Jaw control with A (south) and Y (north) buttons
        jaw_vel = 0.0
        if msg.buttons[SOUTH_BUTTON]:  # A button - close jaw (negative velocity)
            jaw_vel = -self.jaw_vel_scale
        if msg.buttons[NORTH_BUTTON]:  # Y button - open jaw (positive velocity)
            jaw_vel = self.jaw_vel_scale

        if jaw_vel != 0.0:
            jaw_msg = ArmValues()
            jaw_msg.header.stamp = self.get_clock().now().to_msg()
            jaw_msg.joints = [float('nan')] * 6
            jaw_msg.jaw = jaw_vel
            self.jaw_pub.publish(jaw_msg)

    def twist_msg_is_zero(self, msg):
        t = msg.twist
        return (
            t.linear.x == 0.0
            and t.linear.y == 0.0
            and t.linear.z == 0.0
            and t.angular.x == 0.0
            and t.angular.y == 0.0
            and t.angular.z == 0.0
        )


def main():
    try:
        rclpy.init()
        node = GamepadControl()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
