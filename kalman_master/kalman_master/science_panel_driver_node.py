import yaml

import rclpy

from rclpy.node import Node

from kalman_interfaces.msg import MasterMessage


class SciencePanelDriver(Node):
    def __init__(self):
        super().__init__("ueuos_driver")
        filename = self.declare_parameter("config_path").value

        with open(filename, "r") as f:
            self.panels_list = yaml.safe_load(f)

        # Init master publisher.
        self.ueuos_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )


def main():
    try:
        rclpy.init()
        node = SciencePanelDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
