from typing import Optional
from rclpy.node import Node
from kalman_interfaces.msg import ArmFkCommand, ArmIkCommand
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Float64

# TODO split into two services one for fk and one for ik


class CommandBridge:
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        self.sub_fk = parent_node.create_subscription(
            ArmFkCommand, "/station/arm/fk/command", self.handler_fk, qos_profile=10
        )
        self.sub_ik = parent_node.create_subscription(
            ArmIkCommand, "/station/arm/ik/command", self.handler_ik, qos_profile=10
        )

        self.pub_ik = parent_node.create_publisher(
            TwistStamped, "/servo_server/delta_twist_cmds", qos_profile=1
        )

        self.pub_fk = parent_node.create_publisher(
            JointJog, "/servo_server/delta_joint_cmds", qos_profile=1
        )

        self.pub_gripper = parent_node.create_publisher(
            Float64, "/gripper_controller/incremental" , qos_profile=1
        )

    def handler_fk(self, message: ArmFkCommand):
        msg = JointJog()
        msg.header.stamp = self.parent_node.get_clock().now()

        joint_names = []
        joint_velocities = []

        for name in [f"joint_{i}" for i in range(1, 7)]:
            velocity = getattr(message, name)

            if name == "joint_5":
                velocity = -velocity

            if velocity != 0.0:
                joint_names.append(name)
                joint_velocities.append(velocity * 0.33)

        if hasattr(message, "gripper"):
            self.send_gripper_command(getattr(message, "gripper"))

        msg.joint_names = joint_names
        msg.velocities = joint_velocities

        self.pub_fk.publish(msg)

    def handler_ik(self, message: ArmIkCommand):
        msg = TwistStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.parent_node.get_clock().now()

        msg.twist.linear.x = message.linear_x * 0.1
        msg.twist.linear.y = message.linear_y * 0.1
        msg.twist.linear.z = message.linear_z * 0.1
        msg.twist.angular.x = message.angular_x * 0.5
        msg.twist.angular.y = message.angular_y * 0.5
        msg.twist.angular.z = message.angular_z * 0.5

        self.pub_ik.publish(msg)

    def send_gripper_command(self, value: Optional[float]):
        msg = Float64()
        msg.data = value * 0.01
        self.pub_gripper.publish(msg)
