from fastapi import APIRouter
from rclpy.node import Node
from .api.configuration import VideoConfigurationRouter

class VideoAPI(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="/video", tags=["video"])
        self.include_router(VideoConfigurationRouter(parent_node))
