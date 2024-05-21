from enum import Enum
from typing import Any, Callable, Dict, Sequence, Type
from typing_extensions import Annotated, Doc, deprecated
from fastapi.datastructures import Default
from fastapi.routing import APIRoute
from fastapi.utils import generate_unique_id
from starlette.responses import JSONResponse
from starlette.routing import BaseRoute
from starlette.types import ASGIApp
import rclpy
from rclpy.node import Node
from fastapi import APIRouter, Response
from ..model.state import ArmState
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Empty
# from arm_hardware_interface.srv import (
#     SetPosition,
#     setpositionrequest,
#     Positioning,
#     PositioningRequest,
# )


class ArmPositioning(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="/positioning", tags=["positioning"])

        ## ROS initialization
        # start_positioning = rospy.ServiceProxy("/start_joint_positioning", Positioning)
        # abort_positioning = rospy.ServiceProxy("/abort_joint_positioning", Empty)
        # set_actual_joint_position = rospy.ServiceProxy(
        #     "/set_actual_joint_position", SetPosition
        # )
        # ros2uart_publisher = rospy.Publisher(
        #     "/kalman_rover/ros2uart", UInt8MultiArray, queue_size=10
        # )

        # Webserver stuff
        self.add_api_route(
            "/start",
            self.start_joint_positioning, # Callable
            name="Start joint positioning",
            response_model=bool,
            response_description="True if succeeded",
            methods=["PUT"],
        )

        self.add_api_route(
            "/abort",
            self.abort_joint_positioning, # Callable
            name="Abort joint positioning",
            response_model=bool,
            response_description="True if succeeded",
            methods=["PUT"],
        )

        self.add_api_route(
            "/set_actual_joint_position",
            self.set_actual_joint_position,
            name="",
            response_model=bool,
            response_description="True if succeeded",
            methods=["PUT"],
        )

        self.add_api_route(
            "/reset_gripper_servo",
            self.reset_gripper_servo,
            name="",
            response_model=bool,
            response_description="True if succeeded",
            methods=["GET"],
        )

    def start_joint_positioning(self, joint_id: int) -> bool:
        # try:
        #     start_positioning(PositioningRequest(joint_id))
        #     return True
        # except Exception as e:
        #     rospy.logerr(e)
        #     return False

        return True

    def abort_joint_positioning(self) -> bool:
        # try:
        #     abort_positioning()
        #     return True
        # except Exception as e:
        #     rospy.logerr(e)
        #     return False

        return True
            

    def set_actual_joint_position(
        self,
        joint_id: int,
        position_radians: float,
    ) -> bool:
        # try:
        #     set_actual_joint_position(
        #         SetPositionRequest(joint_id=joint_id, position=position_radians)
        #     )
        #     return True
        # except Exception as e:
        #     rospy.logerr(e)
        #     return False

        return True


    def reset_gripper_servo(self) -> bool:
        # ros2uart_publisher.publish(UInt8MultiArray(data=[250, 0]))
        # rospy.sleep(0.05)

        return True

