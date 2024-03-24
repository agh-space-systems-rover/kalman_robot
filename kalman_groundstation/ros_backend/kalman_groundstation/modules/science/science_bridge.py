from .bridge.command import CommandBridge
class ScienceBridge:
    def __init__(self):
        self.sub_services = [
            CommandBridge()
        ]