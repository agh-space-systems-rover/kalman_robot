import numpy as np
import rclpy
import rclpy.node
import rclpy.qos
from geometry_msgs.msg import Twist
from kalman_interfaces.msg import WheelStates
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
import time

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


class State:
    pass


class StopState(State):
    """
    Robot is stopped.
    Wheels states are not published.
    """

    pass


class DriveState(State):
    """
    Robot is driving.
    Wheels states are published continuously.
    """

    pass


class AdjustWheelsState(State):
    """
    Robot is adjusting wheels.
    Wheels states are published continuously.
    Velocities are set to 0.
    """

    pass


# This node republishes different motion control messages as a universal wheel
# state message for a robot with quadruple independent swivel wheels.
class WheelController(rclpy.node.Node):
    def __init__(self):
        super().__init__("wheel_controller")

        # Read parameters.
        self.declare_parameter(
            "queue_size",
            10,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="queue size for command subscribers and state publisher",
            ),
        )
        self.declare_parameter(
            "publish_freq",
            10,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="frequency of state publisher (Hz)",
            ),
        )
        self.declare_parameter(
            "stop_timeout",
            2.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="time to wait before automatically stopping wheels after last issued command (s)",
            ),
        )
        self.declare_parameter(
            "robot_radius",
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="radius of the robot in meters; Used to calculate wheel angles from angular velocity. (m)",
            ),
        )
        self.declare_parameter(
            "wheel_turn_vel",
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="angular speed of the swivel when changing wheel orientation; Predicates for how long the robot will wait when adjusting wheels. (rad/s)",
            ),
        )
        self.declare_parameter(
            "max_wheel_turn_diff",
            0.7,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="minimum difference between current and target wheel swivel angle to adjust wheels in place (rad)",
            ),
        )
        self.declare_parameter(
            "min_wheel_turn_diff",
            0.1,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="maximum difference between current and target wheel swivel angle to stop adjusting wheels in place (rad)",
            ),
        )

        # Initialize current wheel states.
        self.state: State = DriveState()
        self.last_cmd_time = 0
        self.target_velocities = np.zeros(4)
        self.target_angles = np.zeros(4)
        self.simulated_angles = np.zeros(4)

        # Create subscribers.
        queue_size = self.get_parameter("queue_size").value
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, queue_size)

        # Create state publisher.
        self.wheel_state_pub = self.create_publisher(
            WheelStates, "/wheel_controller/state", queue_size
        )

        # Create state publisher timer.
        publish_freq = self.get_parameter("publish_freq").value
        self.create_timer(1 / publish_freq, self.publish_state)

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

        # Update target states.
        self.last_cmd_time = time.time()
        self.target_velocities = wheel_velocities
        self.target_angles = wheel_angles

    def publish_state(self):
        # Update simulated wheel angles.
        wheel_turn_vel = self.get_parameter("wheel_turn_vel").value
        publish_freq = self.get_parameter("publish_freq").value
        for i in range(len(self.target_velocities)):
            # Simple PID control.
            diff = self.target_angles[i] - self.simulated_angles[i]
            diff *= 4  # P = 4
            diff = np.clip(diff, -wheel_turn_vel, wheel_turn_vel)
            self.simulated_angles[i] += diff / publish_freq

        # Transfer to the right state.
        time_since_last_cmd = time.time() - self.last_cmd_time
        transferred_to_stop = False
        if isinstance(self.state, StopState):
            # Check if a command has been received.
            # Transfer to drive state if so.
            stop_timeout = self.get_parameter("stop_timeout").value
            if time_since_last_cmd < stop_timeout:
                self.state = DriveState()
        if isinstance(self.state, DriveState):
            # Check if wheels are turned too much.
            # Initiate wheel adjustment if so.
            max_wheel_turn_diff = self.get_parameter("max_wheel_turn_diff").value
            for i in range(len(self.target_angles)):
                if (
                    abs(self.target_angles[i] - self.simulated_angles[i])
                    > max_wheel_turn_diff
                ):
                    self.state = AdjustWheelsState()
                    break

            # Transfer to stop state if no command has been received for a while.
            # This overwrites the previous check.
            stop_timeout = self.get_parameter("stop_timeout").value
            if time_since_last_cmd > stop_timeout:
                self.state = StopState()
                transferred_to_stop = True
        if isinstance(self.state, AdjustWheelsState):
            # Check if wheels are turned enough.
            # Transfer to drive state if so.
            min_wheel_turn_diff = self.get_parameter("min_wheel_turn_diff").value
            for i in range(len(self.target_angles)):
                if (
                    abs(self.target_angles[i] - self.simulated_angles[i])
                    < min_wheel_turn_diff
                ):
                    self.state = DriveState()
                    break

            # Transfer to stop state if no command has been received for a while.
            # This overwrites the previous check.
            stop_timeout = self.get_parameter("stop_timeout").value
            if time_since_last_cmd > stop_timeout:
                self.state = StopState()
                transferred_to_stop = True

        # Publish stop state if transferred to stop state.
        if transferred_to_stop:
            wheel_states = WheelStates()
            wheel_states.front_left.angle = self.target_angles[0]
            wheel_states.front_right.angle = self.target_angles[1]
            wheel_states.back_left.angle = self.target_angles[2]
            wheel_states.back_right.angle = self.target_angles[3]
            self.wheel_state_pub.publish(wheel_states)

        # Process state-specific logic.
        if isinstance(self.state, DriveState):
            # Publish wheel states message.
            wheel_states = WheelStates()
            wheel_states.front_left.velocity = self.target_velocities[0]
            wheel_states.front_left.angle = self.target_angles[0]
            wheel_states.front_right.velocity = self.target_velocities[1]
            wheel_states.front_right.angle = self.target_angles[1]
            wheel_states.back_left.velocity = self.target_velocities[2]
            wheel_states.back_left.angle = self.target_angles[2]
            wheel_states.back_right.velocity = self.target_velocities[3]
            wheel_states.back_right.angle = self.target_angles[3]
            self.wheel_state_pub.publish(wheel_states)

        elif isinstance(self.state, AdjustWheelsState):
            # Publish wheel states message.
            wheel_states = WheelStates()
            wheel_states.front_left.angle = self.target_angles[0]
            wheel_states.front_right.angle = self.target_angles[1]
            wheel_states.back_left.angle = self.target_angles[2]
            wheel_states.back_right.angle = self.target_angles[3]
            self.wheel_state_pub.publish(wheel_states)


def main():
    rclpy.init()

    node = WheelController()
    rclpy.spin(node)

    rclpy.shutdown()
