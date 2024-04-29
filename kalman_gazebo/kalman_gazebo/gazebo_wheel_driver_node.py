import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kalman_interfaces.msg import WheelState, WheelStates

SWIVEL_JOINT_NAMES = [
    "swivel_fr_joint",
    "swivel_br_joint",
    "swivel_bl_joint",
    "swivel_fl_joint",
]

RADIANS_PER_METER = (
    2 / 0.25
)  # = 8; 0.25 m is the wheel diameter; 2 is part of the formula


class GazeboWheelDriverNode(Node):
    def __init__(self):
        super().__init__("wheel_driver")

        self.create_subscription(
            WheelStates, "wheel_states", self.controller_state_received, 10
        )

        self.velocity_pub = self.create_publisher(
            Float64MultiArray, "/velocity_controller/commands", 10
        )
        self.position_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

    def controller_state_received(self, msg: WheelStates):
        wheel_states_in_order: list[WheelState] = [
            msg.front_right,
            msg.back_right,
            msg.back_left,
            msg.front_left,
        ]

        wheels_message = Float64MultiArray()
        wheels_message.data = [
            state.velocity * RADIANS_PER_METER for state in wheel_states_in_order
        ]
        self.velocity_pub.publish(wheels_message)

        swivel_message = JointTrajectory()
        point = JointTrajectoryPoint()
        for i, name in enumerate(SWIVEL_JOINT_NAMES):
            swivel_message.joint_names.append(name)
            point.positions.append(wheel_states_in_order[i].angle)
        swivel_message.points.append(point)
        self.position_pub.publish(swivel_message)


def main():
    rclpy.init()
    node = GazeboWheelDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
