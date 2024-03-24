from typing import Tuple, Optional

from rclpy import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import utm

from .transformer import Transformer

Vec2 = Tuple[float, float]


class Position:
    def __init__(self):
        self.__transformer = Transformer()

        self.__odom: Optional[Vec2] = None
        self.__map: Optional[Vec2] = None

        self.__gps: Optional[Vec2] = None
        self.__alt: Optional[float] = None

        self.__utm: Optional[Vec2] = None
        self.__utm_zone: Optional[Tuple[int, str]] = None
        
        self.__gps_subscriber = Node().create_subscription(NavSatFix, "/gps/fix", self.__gps_callback)
        self.__odom_subscriber = Node().create_subscription(Odometry, "/odometry/filtered", self.__odom_callback)
       
    def __gps_callback(self, msg: NavSatFix):
        self.__gps = (msg.latitude, msg.longitude)
        self.__alt = msg.altitude
        as_utm = utm.from_latlon(msg.latitude, msg.longitude)
        self.__utm = (as_utm[0], as_utm[1])
        self.__utm_zone = (as_utm[2], as_utm[3])

    def __odom_callback(self, msg: Odometry):
        self.__odom = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.__map = self.__transformer.transform2D(self.__odom, "odom", "map")
    
    @property
    def gps(self) -> Optional[Vec2]:
        return self.__gps

    @property
    def alt(self) -> Optional[float]:
        return self.__alt

    @property
    def odom(self) -> Optional[Vec2]:
        return self.__odom

    @property
    def map(self) -> Optional[Vec2]:
        return self.__map

    @property
    def utm(self) -> Optional[Vec2]:
        return self.__utm

    @property
    def utm_zone(self) -> Optional[Tuple[int, str]]:
        return self.__utm_zone