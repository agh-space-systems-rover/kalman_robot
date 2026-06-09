import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

from launch.substitutions import LaunchConfiguration

class TravelDistanceMeterNode(Node):
    def __init__(self):
        super().__init__("travel_distance_meter_node")

        self.declare_parameter("deadzone", 0.01)
        self.deadzone = self.get_parameter("deadzone").get_parameter_value().double_value
        
        publish_period = 1
        self.total_distance = 0.0
        self.last_x = None
        self.last_y = None

        self.odom_sub = self.create_subscription(
            Odometry, "/odometry/local", self.odom_callback, 10
        )

        self.distance_pub = self.create_subscription = self.create_publisher(
            Float32, "/arc/distance_traveled", 10
        )

        self.pub_timer = self.create_timer(publish_period, self.publish_distance)
        self.get_logger().info("Travel Distance Meter Node has been started.")

    def odom_callback(self, msg: Odometry):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        if self.last_x is None or self.last_y is None:
            self.last_x = current_x
            self.last_y = current_y
            return

        dx = current_x - self.last_x
        dy = current_y - self.last_y
        step = math.hypot(dx, dy)

        if step > self.deadzone:
            self.total_distance += step
            self.last_x = current_x
            self.last_y = current_y

    def publish_distance(self):
        msg = Float32()
        msg.data = self.total_distance
        self.distance_pub.publish(msg)


def main():
    try:
        rclpy.init()
        node = TravelDistanceMeterNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
