import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters


class ServoParamSetter(Node):
    def __init__(self):
        super().__init__("servo_param_setter")

        self.servo_linear_sub_ = self.create_subscription(
            Float64, "servo/set_linear_scale", self.linear_callback, 10
        )
        self.servo_rotational_sub_ = self.create_subscription(
            Float64, "servo/set_rotational_scale", self.rotational_callback, 10
        )

        self.client = self.create_client(SetParameters, "/servo_node/set_parameters")

        self.future_linear = None
        self.future_rotational = None

    def linear_callback(self, msg):
        while not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("service not available, waiting again...")

        request = SetParameters.Request()

        param = Parameter()
        param.name = "moveit_servo.scale.linear"

        value = ParameterValue()
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = msg.data
        param.value = value

        request.parameters = [param]

        self.future_linear = self.client.call_async(request)

    def rotational_callback(self, msg):
        while not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("service not available, waiting again...")

        request = SetParameters.Request()

        param = Parameter()
        param.name = "moveit_servo.scale.rotational"

        value = ParameterValue()
        value.type = ParameterType.PARAMETER_DOUBLE
        value.double_value = msg.data
        param.value = value

        request.parameters = [param]

        self.future_rotational = self.client.call_async(request)


def main():
    try:
        rclpy.init()
        node = ServoParamSetter()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
