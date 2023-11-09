#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from master_driver.serial_driver import SerialDriver, SerialMsg
from typing import List

"""
The idea of this node is to provide the ROS <--> Master bridge.

Every frame received from master is published as a ROS message (UInt8MultiArray) on a dynamically 
created topic /master_com/master_to_ros/{0x(lowercase hexadecimal frame ID)} [msg command, argc, argv_0, ... , argv_n], where argc is unsigned, 8-bit variable.
Frame data interpretation should be handled by client.

Every ROS message sent on topic /master_com/ros_to_master should have [msg command, argc, argv_0, ... , argv_n] format.
Message is then encoded as a binary frame and sent out using the serial driver.
"""


class MasterCom(Node):
    # _port_name will be overwritten by launch file
    def __init__(self, port_name: str = "/dev/master3", baud_rate: int = 115200, ascii_mode: bool = False, frequency: float = 400) -> None:
        super().__init__("master_com")

        self.declare_parameter("serial_port", port_name)
        self.declare_parameter("baud_rate", baud_rate)
        self.declare_parameter("ascii_mode", ascii_mode)
        self.declare_parameter("frequency", frequency)

        port_name = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        ascii_mode = self.get_parameter("ascii_mode").value
        frequency = self.get_parameter("frequency").value

        self.get_logger().info(f"serial port:  {port_name}")
        self.get_logger().info(f"baud rate:    {baud_rate}")

        self.driver = SerialDriver(
            self,
            port_name=port_name,
            start_byte="<",
            stop_byte=">",
            baud_rate=baud_rate,
            ascii_mode=ascii_mode,
        )

        self.create_subscription(
            UInt8MultiArray, "/master_com/ros_to_master", self.ros_to_master, 10
        )
        self.pubs = {}

        self.create_timer(1.0 / frequency, self.master_to_ros)

    def master_to_ros(self) -> None:
        self.driver.tick()
        msgs: List[SerialMsg] = self.driver.readAllMsgs()
        for msg in msgs:
            if not msg.cmd in self.pubs:
                self.pubs[msg.cmd] = self.create_publisher(
                    UInt8MultiArray, "/master_com/master_to_ros" + hex(msg.cmd), 10
                )

            ros_msg = UInt8MultiArray()
            data = [msg.cmd, msg.argc]
            data.extend(msg.argv)
            ros_msg.data = data

            self.pubs[msg.cmd].publish(ros_msg)

    def ros_to_master(self, ros_msg: UInt8MultiArray) -> None:
        self.get_logger().info(f"Sending ros_to_master frame: {hex(ros_msg.data[0])}")
        self.driver.writeMsg(ros_msg)
        self.driver.tick()


def main(args=None):
    rclpy.init(args=args)

    node = MasterCom()
    rclpy.spin(node)

    rclpy.shutdown()
