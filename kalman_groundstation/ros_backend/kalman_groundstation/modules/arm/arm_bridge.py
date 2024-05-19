from .bridge.command import CommandBridge
from .bridge.joints import JointStateBridge
from rclpy.node import Node

# TODO split into two services one for fk and one for ik
class ArmBridge:
    def __init__(self, parent_node: Node):
        self.sub_services = [
            CommandBridge(parent_node),
            JointStateBridge(parent_node)
        ]
