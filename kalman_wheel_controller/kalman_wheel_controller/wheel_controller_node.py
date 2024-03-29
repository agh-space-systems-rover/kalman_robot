import numpy as np
import rclpy
import rclpy.node
import rclpy.qos
from geometry_msgs.msg import Twist
from kalman_interfaces.msg import WheelStates
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
import time

# Direction of each wheel in order to turn the robot by a positive angle.
# TURN_VECTORS = [
#     vec / np.linalg.norm(vec)
#     for vec in [
#         np.array([-1, 1]),  # front left
#         np.array([1, 1]),  # front right
#         np.array([-1, -1]),  # back left
#         np.array([1, -1]),  # back right
#     ]
# ]
TURN_VECTORS = [
    vec / np.linalg.norm(vec)
    for vec in [
        np.array([-wheel_pos[1], wheel_pos[0]])  # rot +90
        for wheel_pos in [
            np.array([0.4, 0.33]),  # front left
            np.array([0.4, -0.33]),  # front right
            np.array([-0.4, 0.33]),  # back left
            np.array([-0.4, -0.33]),  # back right
        ]
    ]
]


class State:
    pass


class StopState(State):
    """
    Robot is stopping or stopped.
    Wheel states are not published after slowing down.
    """

    pass


class DriveState(State):
    """
    Robot is driving.
    Wheel states are published continuously.
    """

    pass


class AdjustWheelsState(State):
    """
    Robot is adjusting wheels or slowing down to do so.
    Wheel states are published continuously.
    Velocities gradually decrease to zero.
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
            "rate",
            10,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="update frequency (Hz)",
            ),
        )
        self.declare_parameter(
            "stop_timeout",
            1.0,
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
                description="radius of the robot in meters; Used to calculate wheel angles from angular velocity (m)",
            ),
        )
        self.declare_parameter(
            "max_wheel_vel",
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="maximum wheel velocity (m/s)",
            ),
        )
        self.declare_parameter(
            "wheel_accel",
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="wheel acceleration (m/s^2)",
            ),
        )
        self.declare_parameter(
            "wheel_turn_vel",
            1.57,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="angular speed of the swivel when changing wheel orientation; Predicates for how long the robot will wait when adjusting wheels (rad/s)",
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
            0.2,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="maximum difference between current and target wheel swivel angle to stop adjusting wheels in place (rad)",
            ),
        )

        # Initialize current wheel states.
        self.state: State = DriveState()
        self.last_cmd_time = 0
        self.target_velocities = [0.0, 0.0, 0.0, 0.0]
        self.current_velocities = [0.0, 0.0, 0.0, 0.0]
        self.target_angles = [0.0, 0.0, 0.0, 0.0]
        self.current_angles = [0.0, 0.0, 0.0, 0.0]

        # Create subscribers.
        queue_size = self.get_parameter("queue_size").value
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, queue_size)

        # Create state publisher.
        self.wheel_state_pub = self.create_publisher(
            WheelStates, "/wheel_controller/state", queue_size
        )

        # Create state publisher timer.
        rate = self.get_parameter("rate").value
        self.create_timer(1 / rate, self.publish_state)

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

        # Limit wheel velocities.
        wheel_vel_limit = self.get_parameter("max_wheel_vel").value
        max_wheel_vel = np.max(np.abs(wheel_velocities))
        if max_wheel_vel > wheel_vel_limit:
            scale = wheel_vel_limit / max_wheel_vel
            wheel_velocities = [vel * scale for vel in wheel_velocities]

        # Update target states.
        self.last_cmd_time = time.time()
        self.target_velocities = wheel_velocities
        self.target_angles = wheel_angles

    def publish_state(self):
        # Transfer to the right state.
        time_since_last_cmd = time.time() - self.last_cmd_time
        if isinstance(self.state, StopState):
            # Check if a command has been received.
            # Transfer to drive state if so.
            stop_timeout = self.get_parameter("stop_timeout").value
            if time_since_last_cmd < stop_timeout:
                self.state = DriveState()
        if isinstance(self.state, DriveState):
            # Check if wheels are to be turned too much.
            # Initiate wheel adjustment if so.
            max_wheel_turn_diff = self.get_parameter("max_wheel_turn_diff").value
            for i in range(len(self.target_angles)):
                if (
                    abs(self.target_angles[i] - self.current_angles[i])
                    > max_wheel_turn_diff
                ):
                    self.state = AdjustWheelsState()
                    break

            # Transfer to stop state if no command has been received for a while.
            # This overwrites the previous check.
            stop_timeout = self.get_parameter("stop_timeout").value
            if time_since_last_cmd > stop_timeout:
                self.state = StopState()
        if isinstance(self.state, AdjustWheelsState):
            # Check if wheels are turned enough.
            # Transfer to drive state if so.
            min_wheel_turn_diff = self.get_parameter("min_wheel_turn_diff").value
            all_wheels_in_position = True
            for i in range(len(self.target_angles)):
                if (
                    abs(self.target_angles[i] - self.current_angles[i])
                    > min_wheel_turn_diff
                ):
                    all_wheels_in_position = False
                    break
            if all_wheels_in_position:
                self.state = DriveState()

            # Transfer to stop state if no command has been received for a while.
            # This overwrites the previous check.
            stop_timeout = self.get_parameter("stop_timeout").value
            if time_since_last_cmd > stop_timeout:
                self.state = StopState()

        # Get params and delta time.
        rate = self.get_parameter("rate").value
        dt = 1 / rate

        # Update current wheel velocities while respecting acceleration limits.
        def update_current_velocities(target):
            max_wheel_accel = self.get_parameter("wheel_accel").value
            for i in range(len(self.target_velocities)):
                # Simple P=1 controller.
                # It will precisely converge to the target velocity.
                dv = target[i] - self.current_velocities[i]
                dv = np.clip(dv, -max_wheel_accel * dt, max_wheel_accel * dt)
                self.current_velocities[i] += dv

        # Update current wheel angles.
        def update_current_angles(target):
            wheel_turn_vel = self.get_parameter("wheel_turn_vel").value
            for i in range(len(self.target_angles)):
                dx = target[i] - self.current_angles[i]
                dx = np.clip(dx, -wheel_turn_vel * dt, wheel_turn_vel * dt)
                self.current_angles[i] += dx

        # Publish current states.
        # Use target_angles instead of current_angles, because the robot's wheel turn motors do not experience any high-acceleration issues.
        def publish_current_states(use_target_angles=True):
            vels = self.current_velocities
            angles = self.target_angles if use_target_angles else self.current_angles

            wheel_states = WheelStates()
            wheel_states.front_left.velocity = vels[0]
            wheel_states.front_left.angle = angles[0]
            wheel_states.front_right.velocity = vels[1]
            wheel_states.front_right.angle = angles[1]
            wheel_states.back_left.velocity = vels[2]
            wheel_states.back_left.angle = angles[2]
            wheel_states.back_right.velocity = vels[3]
            wheel_states.back_right.angle = angles[3]
            self.wheel_state_pub.publish(wheel_states)

        # Process state-specific logic.
        if isinstance(self.state, StopState):
            # Publish state only if the robot is still moving.
            if not np.all(np.isclose(self.current_velocities, 0)):
                # Gradually slow down.
                update_current_velocities([0.0, 0.0, 0.0, 0.0])
                publish_current_states(use_target_angles=False)

        elif isinstance(self.state, DriveState):
            # Follow target velocities and angles within the safety limits.
            update_current_velocities(self.target_velocities)
            update_current_angles(self.target_angles)

            publish_current_states()

        elif isinstance(self.state, AdjustWheelsState):
            # Gradually slow down.
            update_current_velocities([0.0, 0.0, 0.0, 0.0])

            # Only adjust wheel angles if the robot is stopped.
            if np.all(np.isclose(self.current_velocities, 0)):
                self.current_velocities = [0.0, 0.0, 0.0, 0.0]
                update_current_angles(self.target_angles)
                publish_current_states()
            else:
                publish_current_states(use_target_angles=False)


def main():
    try:
        rclpy.init()
        node = WheelController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
