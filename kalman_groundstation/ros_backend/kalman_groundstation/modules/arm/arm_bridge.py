from .bridge.command import CommandBridge
from .bridge.joints import JointStateBridge

# TODO split into two services one for fk and one for ik
class ArmBridge:
    def __init__(self):
        self.sub_services = [
            CommandBridge(),
            JointStateBridge()
        ]