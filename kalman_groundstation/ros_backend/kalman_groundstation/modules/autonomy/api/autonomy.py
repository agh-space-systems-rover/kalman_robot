from queue import Empty
from fastapi import APIRouter

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from std_srvs.srv import Empty as EmptySrv
from typing import List
from std_msgs.msg import ColorRGBA

# from kalman_rover_uart.srv import intSrv, intSrvRequest
from kalman_interfaces.srv import SetUeuosColor

MAX_SERVICE_RETRIES = 5


class AutonomyRouter(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="", tags=["autonomy"])

        # Initialize ROS2 related stuffs
        self.parent_node = parent_node
        self.initialize_service_clients()

        self.ros2uart_publisher = parent_node.create_publisher(
            UInt8MultiArray, "/kalman_rover/ros2uart", qos_profile=10
        )

        self.ueuos_colors = {
            0 : ColorRGBA(r=1.0, a=1.0),
            1 : ColorRGBA(r=1.0, a=1.0),
            2 : ColorRGBA(b=1.0, a=1.0),
            3 : ColorRGBA(g=1.0, a=1.0),
        }

        # I love web <3
        self.add_api_route(
            "/autonomy_on_off",
            self.set_autonomy_on_off,  # Callable
            response_model=bool,
            methods=["PUT"],
        )

        self.add_api_route(
            "/clear_costmap",
            self.clear_costmap,  # Callable
            name="Clears the costmap",
            response_model=bool,
            methods=["PUT"],
        )

        self.add_api_route(
            "/max_velocity",
            self.set_max_velocity,  # Callable
            name="Set max robot velocity",
            response_model=bool,
            methods=["PUT"],
        )

        self.add_api_route(
            "/ueuos_state",
            self.set_ueuos_state,  # Callable
            methods=["PUT"],
        )

    def wait_for_service(self, client, logger) -> None:
        retries_so_far = 0
        while (
            (not client.wait_for_service(timeout_sec=1.0))
            and retries_so_far < MAX_SERVICE_RETRIES
        ):
            logger.info(
                f"Waiting for service '{client.srv_name}' ..."
            )
            retries_so_far += 1
        if not client.service_is_ready():
            logger.error(
                f"Failed to wait for service: '{client.srv_name}'"
            )
        else:
            logger.info(
                f"Service '{client.srv_name}' is available"

            )

    def initialize_service_clients(self) -> None:
        self.clear_costmap_client = self.parent_node.create_client(
            EmptySrv, "/move_base/clear_costmaps"
        )
        self.wait_for_service(self.clear_costmap_client, self.parent_node.get_logger())

        self.ueuos_set_state_client = self.parent_node.create_client(
            SetUeuosColor, "/ueuos/set_color"
        )
        self.wait_for_service(self.ueuos_set_state_client, self.parent_node.get_logger())


    def set_autonomy_on_off(self, autonomy_on: bool):
        if autonomy_on:
            self.ros2uart_publisher.publish(UInt8MultiArray(data=[0x20, 0x01, 0x02]))
        else:
            self.ros2uart_publisher.publish(UInt8MultiArray(data=[0x20, 0x01, 0x00]))

    def clear_costmap(self):
        req = EmptySrv.Request()
        future = self.clear_costmap_client.call_async(req)
        rclpy.spin_until_future_complete(self.parent_node, future, timeout_sec=0.1)
        return future.done()

    def set_max_velocity(self, speed: int):
        speed = min(max(speed, 0), 100)
        # self.max_velocity_proxy(intSrvRequest(speed))
        # self.max_velocity_client.call_async()
        ## TODO: implement

    def set_ueuos_state(self, color: int):
        req = SetUeuosColor.Request()
        color_ix = min(max(color, 0), 3)
        req.color = self.ueuos_colors[color_ix]
        future = self.ueuos_set_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.parent_node, future, timeout_sec=0.1)
        return future.done()
