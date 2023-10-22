#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from kalman_base.lightweight_serial_driver import LightweightSerialDriver, UartMsg
from typing import List

'''
The idea of this node is to provide the ROS <--> UART bridge.

Every received uart message is published as a ROS message (UInt8MultiArray) on a dynamicly 
created topic /kalman_rover/uart2ros/{message command} [msg comand, arg counter, argv_0, ... , argv_n], where arg_k is unsigned, 8-bit variable.
Uart frame decoding should be handled by client.

Every ROS message sent on topic /kalman_rover/ros2uart should have [msg comand, arg counter, argv_0, ... , argv_n] format.
Message is then encoded as uart frame and handled by uart driver.
'''


class UartRosBridge(Node):

    # _port_name will be overwritten by launch file
    def __init__(self, port_name='/dev/master3', baud_rate=115200, ascii_mode=False):
        super().__init__('uart_ros_bridge')

        self.declare_parameter('/serial_port', port_name)
        self.declare_parameter('/baud_rate', baud_rate)
        self.declare_parameter('/ascii_mode', ascii_mode)

        port_name = self.get_parameter('/serial_port').value
        baud_rate = self.get_parameter('/baud_rate').value
        ascii_mode = self.get_parameter('/ascii_mode').value

        self.get_logger().info(f"Starting uart_ros_bridge node...")
        self.get_logger().warn(f"serial port:  {port_name}")
        self.get_logger().warn(f"baud rate:    {baud_rate}")

        self.driver = LightweightSerialDriver(
            _port_name=port_name, _START_BYTE='<', _STOP_BYTE='>', _BAUD=baud_rate,ascii_mode=ascii_mode)
        
        self.sub = self.create_subscription(
            UInt8MultiArray, '/kalman_rover/ros2uart', self.ros2uart, 10)
        self.publishers = {}

        UART2ROS_SLEEP_TIME = 0.0025
        self.timer_uart2ros = self.create_timer(
            UART2ROS_SLEEP_TIME, self.uart2ros)
        


    def uart2ros(self):
        self.driver.tick()
        msgs: List[UartMsg] = self.driver.readAllMsgs()
        for msg in msgs:
            if not msg.cmd in self.publishers:
                self.get_logger().warn('Initialising topic = {}'.format(
                    '/kalman_rover/uart2ros/' + str(msg.cmd)))
                self.publishers[msg.cmd] = self.create_publisher(
                    UInt8MultiArray, '/kalman_rover/uart2ros/' + str(msg.cmd), 10)
                
            ros_msg = UInt8MultiArray()
            data = [msg.cmd, msg.argc]
            data.extend(msg.argv)
            ros_msg.data = data

            self.publishers[msg.cmd].publish(ros_msg)

    def ros2uart(self, ros_msg):
        self.driver.writeMsg(ros_msg)
        self.driver.tick()


def main(args=None):
    rclpy.init(args=args)

    uart_ros_bridge = UartRosBridge()
    uart_ros_bridge.get_logger().info("uart_ros_bridge node started")

    rclpy.spin(uart_ros_bridge)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

