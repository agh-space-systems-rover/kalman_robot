import yaml
import rospkg
import os
# import rospy
from std_msgs.msg import UInt8MultiArray, Float32
from kalman_interfaces.msg import MasterMessage, ScienceState as ScienceStateMsg
from ..model.science import ScienceState
from rclpy.node import Node

import struct

from kalman_groundstation.modules.science.universal_module import (
    CAN_CMD_WEIGHT_RESPONSE,
    WeightValueFrame,
)

class WeightService:
    def __init__(self, parent_node: Node):
        self.parent_node = parent_node
        self.uart2ros_sub = parent_node.create_subscription(
            MasterMessage, f"/master_com/master_to_ros/x{hex(CAN_CMD_WEIGHT_RESPONSE)[1:]}", self.update_weight, qos_profile=10
        )

        self.weight_publisher = parent_node.create_publisher(
            Float32, "/station/science/weight",
            qos_profile=10
        )

    def update_weight(self, msg):
        frame = WeightValueFrame.parse(msg.data)
        raw_weight = frame.adc_value
        if raw_weight == 0:
            self.parent_node.get_logger().warning("Science WeightService - got 0 adc_value")


        # rospy.logerr(raw_weight)
        # load raw_zero from file
        r = rospkg.RosPack()
        pkg_path = r.get_path('kalman_groundstation')
        current_cfg_path: str = os.path.join(
            pkg_path, f'cfg/science/current_weight.yaml')
        tared_cfg_path: str = os.path.join(
            pkg_path, f'cfg/science/tared_weight.yaml')
        raw_zero = -1

        with open(current_cfg_path, 'w') as stream:
            yaml.safe_dump({'weight': raw_weight}, stream)

        with open(tared_cfg_path, 'r') as stream:
            raw_zero = yaml.safe_load(stream)['weight']

        raw_per_gram = 780
        grams = (raw_weight - raw_zero) // raw_per_gram
        self.weight_publisher.publish(grams)
