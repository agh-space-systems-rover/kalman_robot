import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import UInt8MultiArray
from struct import pack

from kalman_interfaces.msg import WheelStates

# TODO change shit below, move to another file
CMD_MOTOR_SET_WHEELS = 0x40
CMD_AUTONOMY_SWITCH = 0x20
METRIC_VELOCITY_TO_MOTOR_VALUE_FACTOR = 1

class WheelDriver(Node):
    def __init__(self):
        super().__init__('wheel_driver')
        self.create_subscription(WheelStates, '/wheel_controller/state', self.controller_state_received, 1)
        self.publisher = self.create_publisher(UInt8MultiArray, '/master_com/ros_to_master', 10)

        self.autonomy_switch(True)

    def controller_state_received(self, msg: WheelStates):
        data = [int(x) for x in [CMD_MOTOR_SET_WHEELS, 0x08, 
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

    def autonomy_switch(self, on: bool):
        data = [CMD_AUTONOMY_SWITCH, 0x01, int(on)*2]
        self.publisher.publish(UInt8MultiArray(data=data))


def main(args=None):
    rclpy.init(args=args)
    wheelDrv = WheelDriver()
    rclpy.spin(wheelDrv)
    rclpy.shutdown()


if __name__ == '__main__':
    main()