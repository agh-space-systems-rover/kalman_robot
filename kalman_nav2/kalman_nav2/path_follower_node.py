import numpy as np
import rclpy
import rclpy.node
import rclpy.qos
import tf2_ros
import quaternion
import time
from service_based_nav2_controller_srvs.srv import ComputeVelocityCommands
from geometry_msgs.msg import TwistStamped, PointStamped
from rcl_interfaces.msg import ParameterType, ParameterDescriptor


BINARY_SEARCH_STEPS = 10


class State:
    pass


class FollowState(State):
    pass


class RotateInPlaceState(State):
    pass


# P is at 0,0
def closest_point_on_segment(v1, v2):
    if np.dot(v1, v1 - v2) < 0:
        return v1
    if np.dot(v2, v2 - v1) < 0:
        return v2
    # https://gdbooks.gitbooks.io/3dcollisions/content/Chapter1/closest_point_on_line.html
    return v1 + np.dot(-v1, v2 - v1) / np.dot(v2 - v1, v2 - v1) * (v2 - v1)


def smoothstep(edge0, edge1, x):
    x = np.clip((x - edge0) / (edge1 - edge0), 0, 1)
    return x * x * (3 - 2 * x)


def angular_velocity_curve(angle_error, max_vel):
    # return np.clip(angle_error, -max_vel, max_vel)
    # return (2 / np.pi) * np.power(np.abs(angle_error), 2) * np.sign(angle_error) * max_vel
    if angle_error > 0:
        ss = smoothstep(0, np.pi / 4, angle_error) * max_vel
    else:
        ss = -smoothstep(0, np.pi / 4, -angle_error) * max_vel

    lin = np.clip(angle_error / (np.pi / 4), -1, 1) * max_vel
    return lerp(ss, lin, 0.3)


def lerp(a, b, t):
    return a + t * (b - a)


def linear_regression(x, y):
    A = np.vstack([x, np.ones(len(x))]).T
    B = y.reshape(-1, 1)
    X = np.linalg.pinv(A) @ B
    return (X[0], X[1])


def linear_rss(x, y, a, b):
    return np.sum(np.square(y - (a * x + b)))


def fast_inv_uniform_scale_ortho_mat3(m):
    scale = np.linalg.norm(m[:3, 0])
    orthonormal = m / scale
    return orthonormal.T * scale


def fast_inv_uniform_scale_transform(m):
    basis = m[:3, :3]
    origin = m[:3, 3].reshape(-1, 1)
    inv_basis = fast_inv_uniform_scale_ortho_mat3(basis)
    inv_origin = inv_basis @ -origin
    return np.append(np.append(inv_basis, inv_origin, axis=1), [[0, 0, 0, 1]], axis=0)


# This node generates twist commands based on the current path and the robot's pose.
class PathFollower(rclpy.node.Node):
    def __init__(self):
        super().__init__("path_follower")

        self.declare_parameter(
            "robot_frame",
            "base_link",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="the frame of the robot",
            ),
        )
        self.declare_parameter(
            "lookahead",
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="pure pursuit lookahead distance (m)",
            ),
        )
        self.declare_parameter(
            "regression_distance",
            3.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Inspect path curvature ahead at this distance. Switch to slow_linear_velocity when the path is bending. (m)",
            ),
        )
        self.declare_parameter(
            "approach_distance",
            1.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Switch to slow_linear_velocity when at the end of the path. (m)",
            ),
        )
        self.declare_parameter(
            "approach_linear_velocity",
            0.2,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="how fast the robot moves when approaching the goal (m/s)",
            ),
        )
        self.declare_parameter(
            "slow_linear_velocity",
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="how fast the robot moves at reduced speed (m/s); Speed is reduced when upcoming path is not straight.",
            ),
        )
        self.declare_parameter(
            "fast_linear_velocity",
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="how fast the robot moves at full speed (m/s)",
            ),
        )
        self.declare_parameter(
            "angular_velocity",
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="how fast the robot turns (rad/s); Angular velocity is strongly reduced when the robot is facing the target direction to reduce oscillations.",
            ),
        )
        self.declare_parameter(
            "rotate_in_place_start_angle",
            1.4,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="If the robot is heading in a direction off of the target direction over this margin, it will rotate in place instead of translating simultaneously. (rad)",
            ),
        )
        self.declare_parameter(
            "rotate_in_place_end_angle",
            0.3,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Stop rotating in place when the robot is parallel to the target direction within this margin. (rad)",
            ),
        )
        self.declare_parameter(
            "driving_mode",
            "hybrid",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="driving mode; forward; backward; hybrid",
            ),
        )
        self.declare_parameter(
            "publish_target",
            True,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="publish the target position for debugging",
            ),
        )

        # Initialize TF2 buffer and listener.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Create a service compatible with service_based_nav2_controller.
        self.srv = self.create_service(
            ComputeVelocityCommands,
            "/path_follower/compute_velocity_commands",
            self.compute_velocity_commands_callback,
        )

        # Debug: create target position publisher.
        self.target_pub = self.create_publisher(
            PointStamped, "/path_follower/target_pos", 10
        )

        self.state = FollowState()

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
        robot_frame = self.get_parameter("robot_frame").value
        res.cmd_vel = TwistStamped()
        res.cmd_vel.header.stamp = self.get_clock().now().to_msg()
        res.cmd_vel.header.frame_id = robot_frame

        # Transform to base_link.
        try:
            robot_frame_to_path = self.lookup_tf_as_matrix(
                req.path.header.frame_id, robot_frame, rclpy.time.Time()
            )
            xy = robot_frame_to_path[:2, 3]
            robot_heading = np.array([1.0, 0.0, 0.0, 0.0])  # in robot_frame
            robot_heading_in_path_frame = (robot_frame_to_path @ robot_heading)[:2]
            yaw = np.arctan2(
                robot_heading_in_path_frame[1], robot_heading_in_path_frame[0]
            )
            # NOTE: For some odd reason, np.linalg.inv is causing extremely high CPU usage.
            path_to_robot_frame_2d = fast_inv_uniform_scale_transform(
                np.array(
                    [
                        [np.cos(yaw), -np.sin(yaw), 0.0, xy[0]],
                        [np.sin(yaw), np.cos(yaw), 0.0, xy[1]],
                        [0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 1.0],
                    ]
                )
            )
        except tf2_ros.LookupException as e:
            self.get_logger().error(f"TF2 lookup failed: {e}")
            return res

        # In case of invalid path, stop the robot.
        if len(req.path.poses) == 0:
            return res

        # Search for the closest vertex in the path.
        # All vertices are transformed to robot_frame using a TF transform projected to 2D.
        closest_vertex_index = None
        closest_dist = float("inf")
        path_vertices = []
        for i in range(len(req.path.poses)):
            # Get the position and transform it to robot_frame
            pose = req.path.poses[i]
            pos = np.array(
                [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            )
            pos = (path_to_robot_frame_2d @ np.append(pos, [1.0]))[:3]
            pos = pos[:2]  # project to 2D

            # Remember the index of the point that is the closest to 0,0 (the robot's position)
            dist = np.linalg.norm(pos)
            if dist < closest_dist:
                closest_vertex_index = i
                closest_dist = dist
            path_vertices.append(pos)

        # Compute the closest position on the path.
        # We have to consider two segments: the one between the closest and the next vertex,
        # and the one between the previous and the closest vertex.
        # closest_dist = dist to the closest vertex from previous step
        target_pos = path_vertices[closest_vertex_index]
        prev_index = (
            closest_vertex_index  # Used to optionally travel forwards on the path.
        )
        if closest_vertex_index > 0:
            # We can compute the distance to the previous segment.
            pos = closest_point_on_segment(
                path_vertices[closest_vertex_index - 1],
                path_vertices[closest_vertex_index],
            )
            dist = np.linalg.norm(pos)
            if dist < closest_dist:
                closest_dist = dist
                target_pos = pos
                prev_index = closest_vertex_index - 1
        if closest_vertex_index < len(path_vertices) - 1:
            # We can compute the distance to the next segment.
            pos = closest_point_on_segment(
                path_vertices[closest_vertex_index],
                path_vertices[closest_vertex_index + 1],
            )
            if dist < closest_dist:
                closest_dist = dist
                target_pos = pos
                prev_index = closest_vertex_index

        # Determine whether the path ahead is straight.
        # Perform linear regression on points close to the robot to determine how straight the path is.
        regression_distance = self.get_parameter("regression_distance").value
        regression_points = []
        path_is_straight = False
        for i in range(prev_index, len(path_vertices)):
            if np.linalg.norm(path_vertices[i]) > regression_distance:
                break
            regression_points.append(path_vertices[i])
        if len(regression_points) > 1:
            x = np.array([p[0] for p in regression_points])
            y = np.array([p[1] for p in regression_points])
            a, b = linear_regression(np.array(x), np.array(y))
            rss = linear_rss(x, y, a, b)
            if rss < 0.1:
                path_is_straight = True

        # Travel on the path until a point at the lookahead distance is found.
        # start_index denotes the first vertex before the robot.
        if path_is_straight:
            lookahead = self.get_parameter("lookahead").value
        else:
            lookahead = self.get_parameter("lookahead").value / 2
        if np.linalg.norm(target_pos) < lookahead:  # if inside circle
            while True:
                # Get the next vertex.
                if prev_index + 1 >= len(path_vertices):
                    break

                # Check if the next vertex is within the lookahead distance.
                next_vertex = path_vertices[prev_index + 1]
                if np.linalg.norm(next_vertex) < lookahead:
                    # If yes, continue to the next vertex.
                    prev_index += 1
                    target_pos = next_vertex
                else:
                    # If no, binary search for the optimal position.
                    for _ in range(BINARY_SEARCH_STEPS):
                        mid_pos = (target_pos + next_vertex) / 2
                        if np.linalg.norm(mid_pos) < lookahead:
                            target_pos = mid_pos
                        else:
                            next_vertex = mid_pos
                    break

        # Debug: publish the target position.
        if self.get_parameter("publish_target").value:
            # target_pos_inv_transformed = (np.linalg.inv(path_to_robot_frame_2d) @ np.append(target_pos, [1.0]))[:2]
            target_pos_msg = PointStamped()
            target_pos_msg.header.stamp = self.get_clock().now().to_msg()
            target_pos_msg.header.frame_id = robot_frame
            target_pos_msg.point.x = target_pos[0]
            target_pos_msg.point.y = target_pos[1]
            self.target_pub.publish(target_pos_msg)

        # Compute the target direction.
        target_dir = target_pos / np.linalg.norm(target_pos)

        # Find the heading of the robot.
        driving_mode = self.get_parameter("driving_mode").value
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
        if isinstance(self.state, FollowState):
            # Check if approaching the goal.
            approach_distance = self.get_parameter("approach_distance").value
            distance_to_end_of_path = np.linalg.norm(path_vertices[-1])

            # Choose the appropriate speed.
            if distance_to_end_of_path < approach_distance:
                linear_velocity = self.get_parameter("approach_linear_velocity").value
            elif path_is_straight:
                linear_velocity = self.get_parameter("fast_linear_velocity").value
            else:
                linear_velocity = self.get_parameter("slow_linear_velocity").value

            # Drive in the target direction.
            res.cmd_vel.twist.linear.x = target_dir[0] * linear_velocity
            res.cmd_vel.twist.linear.y = target_dir[1] * linear_velocity

            # Simultaneously try to minimize the angle between the heading and target direction.
            angular_velocity = self.get_parameter("angular_velocity").value
            res.cmd_vel.twist.angular.z = angular_velocity_curve(
                rot_angle, angular_velocity
            )

            # If the robot is heading in a direction off of the target
            # direction over rotate_in_place_start_angle, it will rotate
            # in place instead of translating simultaneously.
            if (
                np.abs(rot_angle)
                > self.get_parameter("rotate_in_place_start_angle").value
            ):
                self.state = RotateInPlaceState()
        elif isinstance(self.state, RotateInPlaceState):
            # Rotate in place to face the target direction.
            angular_velocity = self.get_parameter("angular_velocity").value
            res.cmd_vel.twist.angular.z = np.sign(rot_angle) * angular_velocity

            # Transition to RotateInPlace when the robot is almost parallel to the target direction.
            if (
                np.abs(rot_angle)
                < self.get_parameter("rotate_in_place_end_angle").value
            ):
                self.state = FollowState()
        else:
            raise ValueError(f"Invalid state: {self.state}")

        return res


def main():
    try:
        rclpy.init()
        node = PathFollower()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
