import numpy as np
import rclpy
import rclpy.node
import rclpy.qos
import std_msgs.msg
from geometry_msgs.msg import Twist
from kalman_interfaces.msg import WheelStates, WheelState

# Direction of each wheel in order to turn the robot by a positive angle.
TURN_VECTORS = [
    vec / np.linalg.norm(vec)
    for vec in [
        np.array([-1, 1]),  # front left
        np.array([1, 1]),  # front right
        np.array([-1, -1]),  # back left
        np.array([1, -1]),  # back right
    ]
]


# This node republishes different motion control messages as a universal wheel
# state message for a robot with quadruple independent swivel wheels.
class WheelController(rclpy.node.Node):
    def __init__(self):
        super().__init__("wheel_controller")

        # Read parameters.
        self.declare_parameter("queue_size", 10)
        self.declare_parameter("robot_radius", 0.5)
        self.queue_size = self.get_parameter("queue_size").value

        # Create subscribers.
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, self.queue_size)

        # Create state publisher.
        self.wheel_state_pub = self.create_publisher(
            WheelStates, "/wheel_controller/state", self.queue_size
        )

    def on_cmd_vel(self, msg: Twist):
        # linear motion vector
        linear_vector = np.array([msg.linear.x, msg.linear.y])

        # angular motion vectors
        robot_radius = self.get_parameter("robot_radius").value
        angular_vectors = [vec * msg.angular.z * robot_radius for vec in TURN_VECTORS]

        # final wheel vectors
        wheel_vectors = [
            linear_vector + angular_vector for angular_vector in angular_vectors
        ]

        # wheel velocities
        wheel_velocities = [np.linalg.norm(vec) for vec in wheel_vectors]

        # wheel angles
        wheel_angles = [np.arctan2(vec[1], vec[0]) for vec in wheel_vectors]

        # Flip wheel velocities and angles if the absolute angle exceeds 90 degrees.
        for i in range(len(wheel_angles)):
            if wheel_angles[i] < -np.pi / 2:
                wheel_angles[i] += np.pi
                wheel_velocities[i] *= -1
            elif wheel_angles[i] > np.pi / 2:
                wheel_angles[i] -= np.pi
                wheel_velocities[i] *= -1

        # Publish wheel states.
        wheel_states = WheelStates()
        wheel_states.front_left = WheelState()
        wheel_states.front_left.velocity = wheel_velocities[0]
        wheel_states.front_left.angle = wheel_angles[0]
        wheel_states.front_right = WheelState()
        wheel_states.front_right.velocity = wheel_velocities[1]
        wheel_states.front_right.angle = wheel_angles[1]
        wheel_states.back_left = WheelState()
        wheel_states.back_left.velocity = wheel_velocities[2]
        wheel_states.back_left.angle = wheel_angles[2]
        wheel_states.back_right = WheelState()
        wheel_states.back_right.velocity = wheel_velocities[3]
        wheel_states.back_right.angle = wheel_angles[3]
        self.wheel_state_pub.publish(wheel_states)


def main():
    rclpy.init()

    node = WheelController()
    rclpy.spin(node)

    rclpy.shutdown()
