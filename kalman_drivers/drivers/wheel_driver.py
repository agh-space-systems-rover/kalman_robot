import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import UInt8MultiArray
from struct import pack

from kalman_interfaces.msg import WheelState, WheelStates

# TODO change shit below, move to another file
JETSON_CMD_MOTOR_SET_WHEELS = 0x40
METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR = 1

class wheel_driver(Node):
    def __init__(self):
        super().__init__('wheel_driver')
        self.create_subscription(WheelStates, '/wheel_controller/state', self.controller_state_received, 1)
        self.publisher = self.create_publisher(UInt8MultiArray, '/ros2uart', 10)

    def controller_state_received(self, msg):
        data = [int(x) for x in [JETSON_CMD_MOTOR_SET_WHEELS, 0x08, 
                                 msg.front_right.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR, 
                                 msg.back_right.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR, 
                                 msg.back_left.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR, 
                                 msg.front_left.velocity * METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR, 
                                 -np.rad2deg(msg.front_right.angle),
                                 np.rad2deg(msg.back_right.angle), 
                                 np.rad2deg(msg.back_left.angle), 
                                 -np.rad2deg(msg.front_left.angle)]]
        data = list(pack('b'*len(data), *data))
        self.publisher.publish(UInt8MultiArray(data=data))


def main(args=None):
    rclpy.init(args=args)
    wheelDrv = wheel_driver()
    rclpy.spin(wheelDrv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()