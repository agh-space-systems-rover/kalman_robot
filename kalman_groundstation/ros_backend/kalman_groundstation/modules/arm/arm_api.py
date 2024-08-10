from fastapi import APIRouter
from rclpy.node import Node

from .api.state import ArmStatePublisher
from .api.goal import ArmGoalRouter
from .api.positioning import ArmPositioning
from .api.trajectories import TrajectoriesAPI
# from .api.positioning import positioning_router

# from .api.trajectories import trajectory_router
from .api.autoclick import AutoclickRouter
from .api.configuration import ArmConfigurationRouter


class ArmAPI(APIRouter):
    def __init__(self, parent_node: Node) -> None:
        super().__init__(prefix="/arm", tags=["arm"])

        self.include_router(ArmStatePublisher(parent_node))
        self.include_router(ArmGoalRouter(parent_node))
        self.include_router(ArmPositioning(parent_node))
        self.include_router(TrajectoriesAPI(parent_node))
        self.include_router(ArmConfigurationRouter(parent_node))
        self.include_router(AutoclickRouter(parent_node))


# router = APIRouter(prefix="/arm", tags=["arm"])
# router.include_router(state_router)
# router.include_router(goal_router)
# router.include_router(positioning_router)
# router.include_router(trajectory_router)
# router.include_router(autoclick_router)
# router.include_router(configuration_router)
