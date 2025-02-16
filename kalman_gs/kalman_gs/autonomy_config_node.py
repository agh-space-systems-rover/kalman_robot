import yaml

import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import Device
from kalman_interfaces.srv import GetDevices


class AutonomyConfig(Node):
    def __init__(self):
        super().__init__("autonomy_config")

        filename = self.declare_parameter("config_path").value

        with open(filename, "r") as f:
            self.device_list = yaml.safe_load(f)

        self.create_service(GetDevices, "autonomy/get_devices", self.get_devices)

    def get_devices(self, req: GetDevices.Request, res: GetDevices.Response):
        res.devices = [
            Device(id=device["id"], device_name=device["device_name"])
            for key, device in self.device_list["devices"].items()
        ]

        return res


def main():
    try:
        rclpy.init()
        node = AutonomyConfig()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
