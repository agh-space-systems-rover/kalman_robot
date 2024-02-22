import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kalman_interfaces.msg import WheelState, WheelStates

TURN_JOINT_NAMES = [
    "sim_suspension_fr_joint",
    "sim_suspension_br_joint",
    "sim_suspension_bl_joint",
    "sim_suspension_fl_joint",
]


class GazeboWheelDriverNode(Node):
    def __init__(self):
        super().__init__("wheel_driver")

        self.create_subscription(
            WheelStates, "wheel_controller/state", self.controller_state_received, 10
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
        wheels_message.data = [state.velocity for state in wheel_states_in_order]
        self.velocity_pub.publish(wheels_message)

        turn_message = JointTrajectory()
        point = JointTrajectoryPoint()
        for i, name in enumerate(TURN_JOINT_NAMES):
            turn_message.joint_names.append(name)
            point.positions.append(wheel_states_in_order[i].angle)
        turn_message.points.append(point)
        self.position_pub.publish(turn_message)


def main():
    rclpy.init()
    node = GazeboWheelDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
