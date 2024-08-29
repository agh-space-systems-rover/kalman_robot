import yaml
import rclpy
import numpy as np
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener

class FiducialOdometry(Node):
    def __init__(self) -> None:
        super().__init__("fiducial_odometry")

        self.config = self.declare_parameter(
            "config", ""
        )
        self.rate = self.declare_parameter(
            "rate", 1.0
        )
        self.odom_frame = self.declare_parameter(
            "odom_frame", "map"
        )
        self.robot_frame = self.declare_parameter(
            "robot_frame", "base_link"
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
        if self.config.value == "":
            self.get_logger().error("No configuration specified.")
            return TransitionCallbackReturn.ERROR
        with open(self.config.value, "r") as file:
            self.fiducials = yaml.safe_load(file)

        # Create the publisher.
        self.pub = self.create_publisher(Odometry, "odometry", 10)
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

        self.get_logger().info("Activated.")
        return TransitionCallbackReturn.SUCCESS

    def timer_callback(self):
        avg_error = np.zeros(2)
        num_fiducials = 0
        for frame, pos in self.fiducials.items():
            # Lookup TF from odom to fiducial.
            try:
                t = self.tf_buffer.lookup_transform(self.odom_frame, frame, 0)
            except Exception:
                # If the TF lookup fails, it is likely because the fiducial is not visible.
                continue
            # Extract the position from the transform.
            odom_pos = np.array([t.transform.translation.x, t.transform.translation.y])
            # Find odom error.
            error = odom_pos - np.array(pos)
            avg_error += error
            num_fiducials += 1

        # If no fiducials are visible, do nothing.
        if num_fiducials == 0:
            return

        # Compute the average error.
        avg_error /= num_fiducials

        # Get the current transform from odom to robot.
        try:
            t = self.tf_buffer.lookup_transform(self.odom_frame, self.robot_frame, 0)
        except Exception:
            return
        robot_pos = np.array([t.transform.translation.x, t.transform.translation.y])

        # Create the odometry message.
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.robot_frame
        msg.pose.pose.position.x = robot_pos[0] - avg_error[0]
        msg.pose.pose.position.y = robot_pos[1] - avg_error[1]
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
