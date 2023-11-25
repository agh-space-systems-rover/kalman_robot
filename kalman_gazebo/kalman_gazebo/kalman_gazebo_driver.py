#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import List
from struct import unpack

"""
The idea of this node is to provide the ROS <--> Master bridge.

Every frame received from master is published as a ROS message (MasterMessage) on a dynamically 
created topic master_com/master_to_ros/{0x(lowercase hexadecimal frame ID)} [msg command, argv_0, ... , argv_n].
Frame data interpretation should be handled by client.

Every ROS message sent on topic master_com/ros_to_master should have MasterMessage format.
Message is then encoded as a binary frame and sent out using the serial driver.
"""

CMD_MOTOR_SET_WHEELS = 0x40 #FIXME: wyjebać to do pliku gdzieś
METERS_PER_S_TO_RADIANS_PER_S = 0.25

class GazeboDriver(Node):
    # _port_name will be overwritten by launch file
    def __init__(self) -> None:
        super().__init__("master_com")
        self.create_subscription(
            MasterMessage, "master_com/ros_to_master", self.ros_to_master, 10
        )
        self.velocity_pub = self.create_publisher(
                    Float64MultiArray, "/velocity_controller/commands", 10
                )

        self.position_pub = self.create_publisher(
                    JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
                )
        


    def ros_to_master(self, ros_msg: MasterMessage) -> None:
        if(ros_msg.cmd == CMD_MOTOR_SET_WHEELS):
            wheels_message = Float64MultiArray()
            wheels_message.data = [float(unpack('b', x.to_bytes(1,"big"))[0])*METERS_PER_S_TO_RADIANS_PER_S for x in ros_msg.data[:4]]
            self.velocity_pub.publish(wheels_message)
            turn_message = JointTrajectory()
            joint_names = ['propulsion_module_fr_joint','propulsion_module_br_joint','propulsion_module_bl_joint','propulsion_module_fl_joint']
            point = JointTrajectoryPoint()
            for i, name in enumerate(joint_names):
                turn_message.joint_names.append(name)
                point.positions.append(ros_msg.data[4+i]*0.01745329)
            turn_message.points.append(point)
            self.position_pub.publish(turn_message)


def main(args=None):
    rclpy.init(args=args)

    node = GazeboDriver()
    rclpy.spin(node)

    rclpy.shutdown()
