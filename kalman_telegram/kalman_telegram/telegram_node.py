import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class StatusClass(Node):
    def __init__(self):
        super().__init__("telegram_channel")

        self.test_sub = self.create_subscription(
            Empty,
            "ros_to_telegram/test",
            self.handle_test,
            10
        )

        self.message_sub = self.create_subscription(
            String,
            "ros_to_telegram/message",
            self.handle_message,
            10
        )

        self.topic_sub = self.create_subscription(
            String,
            "ros_to_telegram/image",
            self.handle_image,
            10
        )

    def handle_test(self, msg: Empty):
        pass

    def handle_message(self, msg: String):
        pass

    def handle_image(self, msg: String):
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
