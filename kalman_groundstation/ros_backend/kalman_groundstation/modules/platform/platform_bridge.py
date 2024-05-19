# from .bridge.driving import DrivingBridge
# from .bridge.motors import MotorsBridge
# from .bridge.temperature import TemperatureBridge
# from .bridge.digging import DiggingBridge
from rclpy.node import Node

class PlatformBridge:
    def __init__(self, parent_node: Node):
        self.sub_services = [
            # DrivingBridge(parent_node),
            # MotorsBridge(parent_node), # TODO
            # TemperatureBridge(parent_node),
            # DiggingBridge(parent_node)
        ]