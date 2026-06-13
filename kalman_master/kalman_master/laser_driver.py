from __future__ import annotations

import struct

import rclpy
from rclpy.node import Node

from rclpy.publisher import Publisher
from rclpy.subscription import Subscription 

from kalman_interfaces.msg import MasterMessage
from std_msgs.msg import Bool, UInt8, UInt16, Float32, Empty

from dataclasses import dataclass
from typing import Optional, Any, Callable, Tuple, List, Set, Dict

import numpy as np

@dataclass
class Publishers:
    laser_dist_out: Optional[Publisher] = None
@dataclass
class Subscriptions:
    laser_dist_in: Optional[Subscription] = None


class LaserDriverNode(Node):
    def __init__(self):
        super().__init__("laser_driver_node")

        self.__pubs = Publishers()
        self.__subs = Subscriptions()

        self.__pubs.laser_dist_out = self.create_publisher(
            Float32, 
            "arc/laser_distance", 
            10
        )
        self.__subs.laser_dist_in = self.create_subscription(
            MasterMessage, 
            f"master_com/master_to_ros/{hex(MasterMessage.LASER_DIST)[1:]}", 
            self.laser_dist_in_callback, 
            10
        )

    def laser_dist_in_callback(self, msg: MasterMessage) -> None:
        
        if len(msg.data) < 4:
            return None

        if msg.cmd == MasterMessage.LASER_DIST:
            board_id, range_status, distance_mm = struct.unpack("<BBH", bytes(msg.data[:4]))

            if range_status == 0: #to oznacza ze zczytana odleglosc jest ponizej 3000mm
                self.__pubs.laser_dist_out.publish(Float32(data=distance_mm / 1000.0))
            else:
                self.__pubs.laser_dist_out.publish(Float32(data=float('inf'))) #out of range
        




    




