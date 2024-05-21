from enum import Enum
from tkinter import N
from typing import Dict, List, Type
from typing_extensions import Annotated, Doc

from fastapi import APIRouter, HTTPException, Response
from fastapi.datastructures import Default
from fastapi.routing import APIRoute
from fastapi.utils import generate_unique_id
import rospkg
from starlette.responses import JSONResponse
from starlette.routing import BaseRoute
from starlette.types import ASGIApp
import rclpy
from rclpy.node import Node
from ..model.goal import ArmGoals, ArmCartesianSpaceGoal, ArmJointSpaceGoal
from geometry_msgs.msg import Pose
import os
import yaml

CARTESIAN_GOALS_FILE = "cartesian_space_goals.yaml"
JOINTS_GOALS_FILE = "joint_space_goals.yaml"


class ArmGoalRouter(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="/goal", tags=["goal"])

        # Webserver stuff
        self.add_api_route(
            "/",
            self.get, # Callable
            name="Handles trajectories for 6dof arm",
            response_model=ArmGoals,
            response_description="Returns saved 6dof goals for arm",
            methods=["GET"],
        )

        self.add_api_route(
            "/cartesian/plan",
            self.put, # Callable
            name="Sends planning request to Moveit for cartesian goal",
            response_model=bool,
            methods=["GET"],
        )

    def get(self) -> ArmGoals:
        return ArmGoals(cartesian_space_goals=[], joint_space_goals=[])

    def put(self, goal: ArmCartesianSpaceGoal) -> bool:
        return True
        

# @goal_router.put(
#     "/joints/plan",
#     name="Sends planning request to Moveit for joint space goal",
#     response_model=bool,
# )
# async def put(goal: ArmJointSpaceGoal):
#     try:
#         _plan_joints(goal)
#     except KeyError as e:
#         print(e)
#         raise HTTPException(
#             status_code=500, detail=f"{e} \n Check if goal exists and format is correct"
#         )

#     return True


# @goal_router.put(
#     "/cartesian/execute",
#     name="Sends plan&execute request to Moveit for cartesian goal",
#     response_model=bool,
# )
# async def put(goal: ArmCartesianSpaceGoal):
#     try:
#         _plan_cartesian(goal)
#     except KeyError as e:
#         print(e)
#         raise HTTPException(
#             status_code=500, detail=f"{e} \n Check if goal exists and format is correct"
#         )
#     group.go(wait=False)
#     return True


# @goal_router.put(
#     "/joints/execute",
#     name="Sends plan&execute request to Moveit for joint space goal",
#     response_model=bool,
# )
# async def put(goal: ArmJointSpaceGoal):
#     try:
#         _plan_joints(goal)
#     except KeyError as e:
#         print(e)
#         raise HTTPException(
#             status_code=500, detail=f"{e} \n Check if goal exists and format is correct"
#         )

#     group.go(wait=False)
#     return True


# def _plan_cartesian(goal: ArmCartesianSpaceGoal):
#     cartesian_space_goals = _load_goals_file(CARTESIAN_GOALS_FILE)
#     target_pose = Pose()
#     target_pose.position.x = cartesian_space_goals[goal.label]["x"]
#     target_pose.position.y = cartesian_space_goals[goal.label]["y"]
#     target_pose.position.z = cartesian_space_goals[goal.label]["z"]
#     target_pose.orientation.x = cartesian_space_goals[goal.label]["qx"]
#     target_pose.orientation.y = cartesian_space_goals[goal.label]["qy"]
#     target_pose.orientation.z = cartesian_space_goals[goal.label]["qz"]
#     target_pose.orientation.w = cartesian_space_goals[goal.label]["qw"]

#     group.set_pose_reference_frame(cartesian_space_goals[goal.label]["frame_id"])
#     group.set_max_velocity_scaling_factor(goal.velocity_scaling_factor)
#     group.set_max_acceleration_scaling_factor(goal.acceleration_scaling_factor)
#     group.set_pose_target(target_pose)
#     group.plan()


# def _plan_joints(goal: ArmJointSpaceGoal):
#     joint_space_goals = _load_goals_file(JOINTS_GOALS_FILE)
#     target_positions = [
#         joint_space_goals[goal.label]["joint_1"],
#         joint_space_goals[goal.label]["joint_2"],
#         joint_space_goals[goal.label]["joint_3"],
#         joint_space_goals[goal.label]["joint_4"],
#         joint_space_goals[goal.label]["joint_5"],
#         joint_space_goals[goal.label]["joint_6"],
#     ]

#     group.set_max_velocity_scaling_factor(goal.velocity_scaling_factor)
#     group.set_max_acceleration_scaling_factor(goal.acceleration_scaling_factor)
#     group.set_joint_value_target(target_positions)
#     group.plan()


# def _load_goals_file(filename: str) -> dict:
#     r = rospkg.RosPack()
#     pkg_path = r.get_path("kalman_groundstation")
#     goals_path: str = os.path.join(pkg_path, f"cfg/arm/{filename}")
#     goals: dict = None
#     with open(goals_path, "r") as stream:
#         goals = yaml.safe_load(stream)
#     return goals
