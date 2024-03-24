from fastapi import APIRouter
from rclpy.node import Node

from .api.science import ScienceRouter

class ScienceAPI(APIRouter):
    def __init__(self, parent_node: Node) -> None:
        super().__init__(prefix="/science")
        self.include_router(ScienceRouter(parent_node))
