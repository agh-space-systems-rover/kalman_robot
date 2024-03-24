from fastapi import APIRouter
from rclpy.node import Node
from .api.autonomy import AutonomyRouter

class AutonomyAPI(APIRouter):
    def __init__(self, parent_node: Node) -> None:
        super().__init__(prefix="/autonomy", tags=["autonomy"])

        self.include_router(AutonomyRouter(parent_node))
