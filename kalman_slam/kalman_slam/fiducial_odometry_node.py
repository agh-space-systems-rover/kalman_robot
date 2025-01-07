import yaml
import rclpy
import numpy as np
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.time import Duration

class FiducialOdometry(Node):
    def __init__(self) -> None:
        super().__init__("fiducial_odometry")

        self.fiducials_path = self.declare_parameter(
            "fiducials_path", ""
        )
        self.rate = self.declare_parameter(
            "rate", 3.0
        ) # Look for new TFs at this rate.
        self.time_offset = self.declare_parameter(
            "time_offset", 0.5
        ) # Look at TFs from that much seconds ago.
        self.odom_frame = self.declare_parameter(
            "odom_frame", "map"
        )
        self.robot_frame = self.declare_parameter(
            "robot_frame", "base_link"
        )
        self.variance = self.declare_parameter(
            "variance", 1.0
        )
        self.max_dist = self.declare_parameter(
            "max_dist", 5.0
        )
        self.max_num = self.declare_parameter(
            "max_num", 1
        )

        result = self.trigger_configure()
        if result != TransitionCallbackReturn.SUCCESS:
            self.get_logger().error("Failed to auto-configure.")
            return

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # Load the config.
        if self.fiducials_path.value == "":
            self.get_logger().error("No fiducials provided.")
            return TransitionCallbackReturn.ERROR
        with open(self.fiducials_path.value, "r") as file:
            try:
                self.fiducials = yaml.safe_load(file)
            except yaml.YAMLError as e:
                self.get_logger().error(f"Failed to load fiducials: {e}")
                return TransitionCallbackReturn.ERROR
            
        self.get_logger().info(f"Loaded {len(self.fiducials)} fiducials.")

        # Create the publisher.
        self.pub = self.create_publisher(Odometry, "odometry", 10)
        self.timer = self.create_timer(1.0 / self.rate.value, self.timer_callback)
        self.published_first_odometry = False

        self.get_logger().info("Activated.")
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        lookup_time = self.get_clock().now() - Duration(nanoseconds=int(self.time_offset.value * 1e9))

        # Get the current transform from odom to robot.
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame.value, self.robot_frame.value, lookup_time)
        except Exception as e:
            self.get_logger().error(f"Failed to lookup transform: {e}")
            return
        robot_pos = np.array([t.transform.translation.x, t.transform.translation.y])

        # Calculate distances from robot to each fiducial and store in a list.
        valid_fiducials = []
        for frame, pos in self.fiducials.items():
            try:
                # Lookup TF from odom to fiducial.
                t = self.tf_buffer.lookup_transform(self.odom_frame.value, frame, lookup_time)
            except Exception as e:
                # If the TF lookup fails, it is likely because the fiducial is not visible.
                continue

            # Extract the position from the transform.
            odom_pos = np.array([t.transform.translation.x, t.transform.translation.y])
            distance = np.linalg.norm(odom_pos - robot_pos)

            # Store only fiducials within the max distance.
            if distance <= self.max_dist.value:
                valid_fiducials.append((frame, pos, odom_pos, distance))

        # If no valid fiducials are found, return early.
        if not valid_fiducials:
            return

        # Sort fiducials by distance (ascending) and limit to max_num.
        valid_fiducials.sort(key=lambda x: x[3])  # Sort by distance (4th item in tuple)
        if self.max_num.value > 0:
            valid_fiducials = valid_fiducials[:self.max_num.value]  # Take closest `max_num` fiducials

        # Compute the error using only the closest fiducials.
        avg_error = np.zeros(2)
        for frame, pos, odom_pos, distance in valid_fiducials:
            # Find odom error.
            error = odom_pos - np.array(pos)
            avg_error += error
        avg_error /= len(valid_fiducials)

        # Create the odometry message.
        msg = Odometry()
        msg.header.stamp = lookup_time.to_msg()
        msg.header.frame_id = self.odom_frame.value
        msg.child_frame_id = self.robot_frame.value
        msg.pose.pose.position.x = robot_pos[0] - avg_error[0]
        msg.pose.pose.position.y = robot_pos[1] - avg_error[1]
        if self.published_first_odometry:
            msg.pose.covariance[0] = self.variance.value
            msg.pose.covariance[7] = self.variance.value
        else:
            self.published_first_odometry = True
        self.pub.publish(msg)


    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # Destroy the publisher.
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.pub.pub_tf)

        self.get_logger().info("Deactivated.")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        return TransitionCallbackReturn.SUCCESS


def main(auto_activate=False):
    try:
        rclpy.init()
        node = FiducialOdometry()

        if auto_activate:
            result = node.trigger_activate()
            if result != TransitionCallbackReturn.SUCCESS:
                node.get_logger().error(f"Failed to auto-activate.")

        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
