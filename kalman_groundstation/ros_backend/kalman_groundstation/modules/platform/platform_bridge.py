from .bridge.driving import DrivingBridge
from .bridge.motors import MotorsBridge
from .bridge.temperature import TemperatureBridge
from .bridge.digging import DiggingBridge

class PlatformBridge:
    def __init__(self):
        self.sub_services = [
            DrivingBridge(),
            MotorsBridge(),
            TemperatureBridge(),
            DiggingBridge()
        ]