# import rospy

from fastapi import APIRouter
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue

class ArmConfigurationRouter(APIRouter):
    def __init__(self, parent_node: Node):
        super().__init__(prefix="/configuration", tags=["configuration"])

        self.node = parent_node
        self.servo_client = self.node.create_client(SetParameters, "/servo_node/set_parameters")
        # Webserver stuff
        self.add_api_route(
            "/linear_vel_scale",
            self.put_linear_vel_scale, # Callable
            name="Sets linear velocity multiplier",
            response_model=bool,
            methods=["GET"],
        )

        self.add_api_route(
            "/angular_vel_scale",
            self.put_angular_vel_scale, # Callable
            name="Sets angular velocity multiplier",
            response_model=bool,
            methods=["GET"],
        )

    def put_linear_vel_scale(self, scale: float) -> bool:
        while not self.servo_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().info("service not available, waiting again...")
    
        request = SetParameters.Request()

        param = Parameter()
        param.name = "moveit_servo.scale.linear"

        value = ParameterValue()
        value.type = 2
        value.double_value = scale
        param.value = value

        request.parameters = [param]

        self.servo_client.call_async(request)
        return True
    
    def put_angular_vel_scale(self, scale: float) -> bool:
        while not self.servo_client.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().info("service not available, waiting again...")
    
        request = SetParameters.Request()

        param = Parameter()
        param.name = "moveit_servo.scale.rotational"
        
        value = ParameterValue()
        value.type = 2
        value.double_value = scale
        param.value = value

        request.parameters = [param]

        self.servo_client.call_async(request)
        return True

