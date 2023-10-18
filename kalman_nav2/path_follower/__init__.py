import time
import numpy as np
import rclpy
import rclpy.node
import rclpy.qos
from service_based_nav2_controller_srvs.srv import ComputeVelocityCommands
from geometry_msgs.msg import TwistStamped
import tf2_ros
import quaternion


class State:
    pass


class EnterFollowState(State):
    def __init__(self):
        self.start_time = time.time()


class FollowState(State):
    pass


class EnterRotateInPlaceState(State):
    def __init__(self):
        self.start_time = time.time()


class RotateInPlaceState(State):
    pass


# This node republishes different motion control messages as a universal wheel
# state message for a robot with quadruple independent swivel wheels.
class PathFollower(rclpy.node.Node):
    def __init__(self):
        super().__init__("path_follower")

        # pure pursuit lookahead distance; m
        self.lookahead = self.declare_parameter("lookahead", 1.0)
        # how fast the robot moves; m/s
        self.linear_velocity = self.declare_parameter("linear_velocity", 1.0)
        # how fast the robot turns; rad/s
        self.angular_velocity = self.declare_parameter("angular_velocity", 1.0)
        # If the robot is heading in a direction off of the target
        # direction over this margin, it will rotate
        # in place instead of translating simultaneously.; radians
        self.rotate_in_place_start_angle = self.declare_parameter(
            "rotate_in_place_start_angle", 1.4
        )
        # Stop rotating in place when the robot is parallel to the target direction within this margin.; radians
        self.rotate_in_place_end_angle = self.declare_parameter(
            "rotate_in_place_end_angle", 0.3
        )
        # Wait for the wheels to turn for this amount of time before and after rotating in place.; seconds
        self.rotate_in_place_wait_time = self.declare_parameter(
            "rotate_in_place_wait_time", 0.5
        )
        # driving mode; forward; backward; hybrid
        self.driving_mode = self.declare_parameter("driving_mode", "hybrid")

        # Initialize TF2 buffer and listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a service compatible with service_based_nav2_controller.
        self.srv = self.create_service(
            ComputeVelocityCommands,
            "/path_follower/compute_velocity_commands",
            self.compute_velocity_commands_callback,
        )

        self.state = EnterFollowState()

    def lookup_tf_as_matrix(
        self, target_frame: str, source_frame: str, time: rclpy.time.Time
    ):
        transform = self.tf_buffer.lookup_transform(
            target_frame, source_frame, time, rclpy.duration.Duration(seconds=1.0)
        ).transform
        pos = np.array(
            [transform.translation.x, transform.translation.y, transform.translation.z]
        )
        rot = np.quaternion(
            transform.rotation.w,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
        )
        transform = np.eye(4)
        transform[:3, :3] = quaternion.as_rotation_matrix(rot)
        transform[:3, 3] = pos
        return transform

    def compute_velocity_commands_callback(
        self,
        req: ComputeVelocityCommands.Request,
        res: ComputeVelocityCommands.Response,
    ):
        # Initialize the response.
        res.cmd_vel = TwistStamped()
        res.cmd_vel.header.stamp = self.get_clock().now().to_msg()
        res.cmd_vel.header.frame_id = "base_link"

        # Transform to base_link.
        try:
            path_to_base_link = self.lookup_tf_as_matrix(
                "base_link", req.path.header.frame_id, rclpy.time.Time()
            )
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"TF2 lookup failed: {e}")
            return res

        # In case of invalid path, stop the robot.
        if len(req.path.poses) == 0:
            return res

        # Search for the closest position in the path.
        closest_path_index = None
        closest_dist = float("inf")
        path_positions = []
        for i in range(len(req.path.poses)):
            pose = req.path.poses[i]
            pos = np.array([pose.pose.position.x, pose.pose.position.y])
            pos = (path_to_base_link @ np.append(pos, [0.0, 1.0]))[:2]
            dist = np.linalg.norm(pos)
            if dist < closest_dist:
                closest_path_index = i
                closest_dist = dist
            path_positions.append(pos)
        # Search for the next target position at lookahead distance starting from the closest position.
        lookahead = self.lookahead.get_parameter_value().double_value
        target_pos = None
        for i in range(closest_path_index, len(path_positions)):
            target_pos = path_positions[i]
            if np.linalg.norm(target_pos) > lookahead:
                break
        target_dir = target_pos / np.linalg.norm(target_pos)

        # Find the heading of the robot.
        driving_mode = self.driving_mode.get_parameter_value().string_value
        if driving_mode not in ["forward", "backward", "hybrid"]:
            raise ValueError(f"Invalid driving mode: {driving_mode}")
        if driving_mode == "forward":
            heading = np.array([1.0, 0.0])
        elif driving_mode == "backward":
            heading = np.array([-1.0, 0.0])
        elif driving_mode == "hybrid":
            if np.dot(np.array([1.0, 0.0]), target_dir) > 0:
                heading = np.array([1.0, 0.0])
            else:
                heading = np.array([-1.0, 0.0])

        # The angle to turn in order to face the target direction.
        rot_angle = np.arctan2(target_dir[1], target_dir[0]) - np.arctan2(
            heading[1], heading[0]
        )
        # normalize to [-pi, pi]
        rot_angle = np.arctan2(np.sin(rot_angle), np.cos(rot_angle))

        # Behave differently depending on the state.
        if isinstance(self.state, EnterFollowState):
            # Turn the wheels forwards.
            res.cmd_vel.twist.linear.x = 0.001

            # Transition to Follow after rotate_in_place_wait_time seconds.
            if (
                time.time() - self.state.start_time
                > self.rotate_in_place_wait_time.get_parameter_value().double_value
            ):
                self.state = FollowState()
        elif isinstance(self.state, FollowState):
            # Try to minimize the angle between the heading and the target direction.
            # Simultaneously drive in the target direction.
            linear_velocity = self.linear_velocity.get_parameter_value().double_value
            angular_velocity = self.angular_velocity.get_parameter_value().double_value
            res.cmd_vel.twist.linear.x = target_dir[0] * linear_velocity
            res.cmd_vel.twist.linear.y = target_dir[1] * linear_velocity
            res.cmd_vel.twist.angular.z = rot_angle * angular_velocity

            # If the robot is heading in a direction off of the target
            # direction over rotate_in_place_start_angle, it will rotate
            # in place instead of translating simultaneously.
            if (
                np.abs(rot_angle)
                > self.rotate_in_place_start_angle.get_parameter_value().double_value
            ):
                self.state = EnterRotateInPlaceState()
        elif isinstance(self.state, EnterRotateInPlaceState):
            # Setup wheels for rotating in place.
            res.cmd_vel.twist.angular.z = 0.001

            # Transition to RotateInPlace after rotate_in_place_wait_time seconds.
            if (
                time.time() - self.state.start_time
                > self.rotate_in_place_wait_time.get_parameter_value().double_value
            ):
                self.state = RotateInPlaceState()
        elif isinstance(self.state, RotateInPlaceState):
            # Rotate in place to face the target direction.
            angular_velocity = self.angular_velocity.get_parameter_value().double_value
            res.cmd_vel.twist.angular.z = rot_angle * angular_velocity

            # Transition to EnterRotateInPlace when the robot is almost parallel to the target direction.
            if (
                np.abs(rot_angle)
                < self.rotate_in_place_end_angle.get_parameter_value().double_value
            ):
                self.state = EnterFollowState()
        else:
            raise ValueError(f"Invalid state: {self.state}")

        return res


def main():
    rclpy.init()

    node = PathFollower()
    rclpy.spin(node)

    rclpy.shutdown()
