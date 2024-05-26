# import rospy

from fastapi import APIRouter
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class ArmConfigurationRouter(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="/configuration", tags=["configuration"])

        self.node = parent_node
        self.linear_pub = self.node.create_publisher(Float64, 'servo/set_linear_scale', 10)
        self.rotational_pub = self.node.create_publisher(Float64, 'servo/set_rotational_scale', 10)
        # Webserver stuff
        self.add_api_route(
            "/linear_vel_scale",
            self.put_linear_vel_scale, # Callable
            name="Sets linear velocity multiplier",
            response_model=bool,
            methods=["PUT"],
        )

        self.add_api_route(
            "/angular_vel_scale",
            self.put_angular_vel_scale, # Callable
            name="Sets angular velocity multiplier",
            response_model=bool,
            methods=["PUT"],
        )

    def put_linear_vel_scale(self, scale: float) -> bool:
        self.linear_pub.publish(Float64(data=scale))
        return True
    
    def put_angular_vel_scale(self, scale: float) -> bool:
        self.rotational_pub.publish(Float64(data=scale))
        return True

