from fastapi import APIRouter
from rclpy.node import Node
from kalman_groundstation.modules.radio.api.silent_mode import SilentModeRouter

class RadioAPI(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="/radio", tags=["radio"])
        self.include_router(SilentModeRouter(parent_node))
