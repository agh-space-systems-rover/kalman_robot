from fastapi import APIRouter

from rclpy.node import Node
# from .api.state import state_router

from .api.state import PlatformStateRouter

# router = APIRouter(prefix="/platform")
# router.include_router(state_router)

class PlatformAPI(APIRouter):
    def __init__(self, parent_node: Node) -> None:
        super().__init__(prefix="/platform", tags=["platform"])

        self.include_router(PlatformStateRouter(parent_node))

