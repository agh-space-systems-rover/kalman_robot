import numpy as np
import rclpy
import rclpy.node
import rclpy.qos
from service_based_nav2_controller_srvs.srv import ComputeVelocityCommands
from geometry_msgs.msg import TwistStamped, PointStamped
import tf2_ros
import quaternion
from rcl_interfaces.msg import ParameterType, ParameterDescriptor


BINARY_SEARCH_STEPS = 10


class State:
    pass


class FollowState(State):
    pass


class RotateInPlaceState(State):
    pass


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
            "linear_velocity",
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="how fast the robot moves (m/s)",
            ),
        )
        self.declare_parameter(
            "angular_velocity",
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="how fast the robot turns (rad/s)",
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

        # Create debug publishers.
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

    # P is at 0,0
    def closest_point_on_segment(self, v1, v2):
        if np.dot(v1, v1 - v2) < 0:
            return v1
        if np.dot(v2, v2 - v1) < 0:
            return v2
        # https://gdbooks.gitbooks.io/3dcollisions/content/Chapter1/closest_point_on_line.html
        return v1 + np.dot(-v1, v2 - v1) / np.dot(v2 - v1, v2 - v1) * (v2 - v1)

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
            path_to_robot_frame = self.lookup_tf_as_matrix(
                robot_frame, req.path.header.frame_id, rclpy.time.Time()
            )
            xy = path_to_robot_frame[:2, 3]
            yaw = np.arctan2(path_to_robot_frame[1, 0], path_to_robot_frame[0, 0])
            path_to_robot_frame_2d = np.array(
                [
                    [np.cos(yaw), -np.sin(yaw), xy[0]],
                    [np.sin(yaw), np.cos(yaw), xy[1]],
                    [0, 0, 1],
                ]
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
            pos = np.array([pose.pose.position.x, pose.pose.position.y])
            pos = (path_to_robot_frame_2d @ np.append(pos, [1.0]))[:2]

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
            pos = self.closest_point_on_segment(
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
            pos = self.closest_point_on_segment(
                path_vertices[closest_vertex_index],
                path_vertices[closest_vertex_index + 1],
            )
            if dist < closest_dist:
                closest_dist = dist
                target_pos = pos
                prev_index = closest_vertex_index

        # Travel on the path until a point at the lookahead distance is found.
        # start_index denotes the first vertex before the robot.
        lookahead = self.get_parameter("lookahead").value
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
            # target_pos_inv_transformed = (np.linalg.inv(path_to_base_link_2d) @ np.append(target_pos, [1.0]))[:2]
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
            # Try to minimize the angle between the heading and the target direction.
            # Simultaneously drive in the target direction.
            linear_velocity = self.get_parameter("linear_velocity").value
            angular_velocity = self.get_parameter("angular_velocity").value
            res.cmd_vel.twist.linear.x = target_dir[0] * linear_velocity
            res.cmd_vel.twist.linear.y = target_dir[1] * linear_velocity
            dx = rot_angle
            dx *= 0.5  # *P
            v = np.clip(dx, -angular_velocity, angular_velocity)
            res.cmd_vel.twist.angular.z = v

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

        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
