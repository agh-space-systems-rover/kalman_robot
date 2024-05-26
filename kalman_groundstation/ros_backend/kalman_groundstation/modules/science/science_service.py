from .services.science import ScienceService
from .services.smart_probe import SmartProbeService
from .services.weight import WeightService
from rclpy.node import Node


class Science:
    def __init__(self, parent_node: Node):
        self.sub_services = [
            ScienceService(parent_node),
            SmartProbeService(parent_node),
            WeightService(parent_node)
        ]
