from typing import Optional
import rospy
from kalman_groundstation.msg import ArmFkCommand, ArmIkCommand
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
from std_msgs.msg import Float64

# TODO split into two services one for fk and one for ik


class CommandBridge:
    def __init__(self):
        self.sub_fk = rospy.Subscriber(
            "/station/arm/fk/command", ArmFkCommand, self.handler_fk
        )
        self.sub_ik = rospy.Subscriber(
            "/station/arm/ik/command", ArmIkCommand, self.handler_ik
        )

        self.pub_ik = rospy.Publisher(
            "/servo_server/delta_twist_cmds", TwistStamped, queue_size=1
        )

        self.pub_fk = rospy.Publisher(
            "/servo_server/delta_joint_cmds", JointJog, queue_size=1
        )

        self.pub_gripper = rospy.Publisher(
            "/gripper_controller/incremental", Float64, queue_size=1
        )

    def handler_fk(self, message: ArmFkCommand):
        msg = JointJog()
        msg.header.stamp = rospy.Time.now()

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
        msg.header.stamp = rospy.Time.now()

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
