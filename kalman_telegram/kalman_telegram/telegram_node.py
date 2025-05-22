import rclpy
from rclpy.node import Node


class StatusClass(Node):
    def __init__(self):
        super().__init__("telegram_channel")

        self.get_logger().info("chuj chuj chuj")

        pass


def main():
    try:
        rclpy.init()
        node = StatusClass()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
