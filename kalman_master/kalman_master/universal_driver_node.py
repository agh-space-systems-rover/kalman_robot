import kalman_interfaces
import yaml
import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import struct
import os
from kalman_interfaces.msg import *
from kalman_interfaces.srv import *

NUMBER_OF_RETRIES_PER_CALL = 1
PUBLISH_RATE = 30  # NOTE: only published when there are messages to send
MAX_MESSAGES_IN_QUEUE = 5



class UniversalDriver(Node):
    def __init__(self):
        super().__init__("universal_driver")

        config_directory_path = self.declare_parameter(
            "config_directory_path",
            "",
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Path to the config file.",
            ),
        ).value

        self.configs = {}
        if os.path.isdir(config_directory_path):
            for filename in os.listdir(config_directory_path):
                if filename.endswith(".yaml"):
                    file_path = os.path.join(config_directory_path, filename)
                    with open(file_path, "r") as file:
                        self.configs[filename] = yaml.safe_load(file)
            self.get_logger().info(f"Loaded configs: {list(self.configs.keys())}")
        else:
            self.get_logger().error(f"Config directory path {config_directory_path} is not a directory")

        for filename in self.configs:
            if self.configs[filename]['services_commands'] != None:
                for service_obj in self.configs[filename]['services_commands']:
                    self.init_service(service_obj)
            


        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )


    def init_service(self, service_obj):
        service_type = eval(service_obj['type'].replace('/', '.'))
        self.create_service(service_type, str(service_obj['service']), 
            self.generate_service_response(service_obj['cmd_translations']))




    def generate_service_response(self, cmd_translations):
        cmd_translations = {eval(key): value for key, value in cmd_translations.items()} # convert constant strings to actual constants
        def response(req, res):
            if req.cmd in cmd_translations:
                for msg in cmd_translations[req.cmd]:
                    self.master_pub.publish(MasterMessage(cmd=eval(msg['cmd']), data=msg['data']))
            else:
                self.get_logger().error(f"Unknown command: {req.cmd}")
            return res
        return response



def main():
    try:
        rclpy.init()
        node = UniversalDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
