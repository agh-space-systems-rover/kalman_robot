from .bridge.command import CommandBridge
from rclpy.node import Node

class ScienceBridge:
    def __init__(self, parent_node: Node):
        self.sub_services = [
            CommandBridge(parent_node)
        ]