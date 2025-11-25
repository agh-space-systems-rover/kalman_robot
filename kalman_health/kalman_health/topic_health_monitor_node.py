import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
import yaml


class StatusClass(Node):
    def __init__(self):
        super().__init__("topic_health_monitor")
        self.timer_list = {}
        filename = self.declare_parameter("config_path", "").value

        with open(filename, "r") as f:
            self.config = yaml.safe_load(f)

        self.statuses = [0] * len(self.config["devices"])

        self.status_pub = self.create_publisher(UInt8, "topic_health_status", 10)

        self.publish_data_timer = self.create_timer(
            self.config["frequency"], self.publish_data
        )
        self.import_interfaces()

    def import_interfaces(self) -> None:
        for i, device in enumerate(self.config["devices"]):
            topic = device["topic"]
            topic_type = device["topic_type"]
            timeout = device["timeout"]

            path_items = topic_type.split("/")
            package = ".".join(path_items[:-1])
            class_name = path_items[-1]

            # Import the interface and save it in the cache.
            module = __import__(package, fromlist=[class_name])
            interface = getattr(module, class_name)

            self.create_subscription(
                interface,
                topic,
                lambda msg, idx=i: self.is_data_received(idx),
                10,
            )
            self.timer_list[i] = self.create_timer(
                timeout, lambda idx=i: self.timeout_callback(idx)
            )

    def is_data_received(self, idx: int):
        self.statuses[idx] = 1
        self.timer_list[idx].reset()

    def timeout_callback(self, idx: int):
        self.statuses[idx] = 0

    def publish_data(self):
        status_byte = 0
        for status in self.statuses:
            status_byte = (status_byte << 1) | status
        self.status_pub.publish(UInt8(data=status_byte))


def main():
    try:
        rclpy.init()
        node = StatusClass()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
