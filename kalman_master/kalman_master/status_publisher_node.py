import rclpy
import time
from rclpy.node import Node
from kalman_interfaces.msg import AutonomyStatus
import yaml

Device_Status = [0, 0, 0, 0, 0, 0, 0, 0]


class StatusClass(Node):
    def __init__(self):
        super().__init__("status_publisher")
        self.timer_list = {}
        self.status_pub = self.create_publisher(
            AutonomyStatus, '/autonomy_status', 10)
        filename = self.declare_parameter("config_path").value

        with open(filename, "r") as f:
            self.Device_List = yaml.safe_load(f)

        self.publish_data_timer = self.create_timer(
            self.Device_List["frequency"], self.publish_data)
        self.import_interfaces()

    def import_interfaces(self) -> None:
        for key, device in self.Device_List["devices"].items():
            _id = device["id"]
            msg_type = device["msg_type"]
            topic = device["topic"]
            timeout = device["timeout"]

            path_items = msg_type.split("/")
            package = ".".join(path_items[:-1])
            class_name = path_items[-1]

            # Import the interface and save it in the cache.
            module = __import__(package, fromlist=[class_name])
            interface = getattr(
                module, class_name)

            self.create_subscription(
                interface, topic,
                lambda msg, id=_id: self.is_data_recieved(id),
                10,
            )
            self.timer_list[_id] = self.create_timer(
                timeout, lambda id=_id: self.timeout_callback(id))

    def is_data_recieved(self, id: int):
        Device_Status[id] = 1
        self.timer_list[id].reset()

    def timeout_callback(self, id: int):
        Device_Status[id] = 0

    def publish_data(self):
        msg = AutonomyStatus()
        list_to_byte = int(''.join(map(str, Device_Status)), 2)
        msg.status = list_to_byte
        self.status_pub.publish(msg)


def main():
    try:
        rclpy.init()
        node = StatusClass()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
