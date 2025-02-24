import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from kalman_interfaces.msg import Device
from kalman_interfaces.srv import GetDevices
import yaml

Device_Status = [0, 0, 0, 0, 0, 0, 0, 0]


class StatusClass(Node):
    def __init__(self):
        super().__init__("topic_health_monitor")
        self.timer_list = {}
        self.status_pub = self.create_publisher(
            UInt8, '/topic_health_status', 10)
        filename = self.declare_parameter("config_path").value

        with open(filename, "r") as f:
            self.Device_List = yaml.safe_load(f)

        self.create_service(GetDevices, "topic_health_status/get_devices", self.get_devices)

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
                lambda msg, id=_id: self.is_data_received(id),
                10,
            )
            self.timer_list[_id] = self.create_timer(
                timeout, lambda id=_id: self.timeout_callback(id))

    def is_data_received(self, id: int):
        Device_Status[id] = 1
        self.timer_list[id].reset()

    def timeout_callback(self, id: int):
        Device_Status[id] = 0

    def publish_data(self):
        msg = UInt8()
        list_to_byte = int(''.join(map(str, Device_Status)), 2)
        msg.data = list_to_byte
        self.status_pub.publish(msg)

    def get_devices(self, req: GetDevices.Request, res: GetDevices.Response):
        res.devices = [
            Device(id=device["id"], device_name=device["device_name"])
            for key, device in self.Device_List["devices"].items()
        ]

        return res


def main():
    try:
        rclpy.init()
        node = StatusClass()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
