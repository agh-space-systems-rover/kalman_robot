import rclpy
from fastapi import APIRouter

from ..model.motors import PlatformState

state_router = APIRouter(prefix="/state", tags=["state"])

STATE_PATH = "/station/system/rover/platform/state/"


@state_router.get(
    "/",
    name="Get platform state",
    response_model=PlatformState,
    response_description="Returns current state of platform",
)
async def get():
    # state = rclpy.get_param(STATE_PATH)
    # TODO: Implement
    return PlatformState()
