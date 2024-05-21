import os
import yaml
from fastapi import APIRouter
from kalman_groundstation.utils.logger import GpsLogger

# import rospy
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Vector3
from std_msgs.msg import UInt8MultiArray
import rospkg


class ScienceRouter(APIRouter):
    def __init__(self, parent_node: Node) -> None:
        super().__init__(prefix="", tags=["science"])
        self.parent_node = parent_node

        self.ros2uart_pub = parent_node.create_publisher(
            UInt8MultiArray, "/kalman_rover/ros2uart", qos_profile=10
        )

        self.panorama_trigger_publisher = parent_node.create_publisher(
            Empty, "/panorama_trigger", qos_profile=10
        )

        self.gps_panorama_logger = GpsLogger("gps_panorama")
        self.gps_user_marker_logger = GpsLogger("gps_user_marker")

        self.add_api_route(
            "/lamp_pwm",
            self.lamp_pwm,
            name="Set PWM for turning on lamp",
            methods=["PUT"],
        )
        self.add_api_route(
            "/log_user_marker",
            self.log_user_marker,
            name="Log GPS of user marker",
            methods=["PUT"],
        )
        self.add_api_route(
            "/get_panorama", self.get_panorama, name="Take panorama", methods=["PUT"]
        )
        self.add_api_route(
            "/get_smart_probe", self.get_smart_probe, name="Get weight", methods=["PUT"]
        )
        self.add_api_route(
            "/tare_weight", self.tare_weight, name="Tare weight", methods=["PUT"]
        )
        self.add_api_route(
            "/get_weight", self.get_weight, name="Get weight", methods=["PUT"]
        )
        self.add_api_route(
            "/open_close_sample",
            self.open_close_sample,
            name="Open and close lid",
            methods=["PUT"],
        )
        self.add_api_route(
            "/set_carousel", self.set_carousel, name="Set carousel", methods=["PUT"]
        )
        self.add_api_route(
            "/set_servo", self.set_servo, name="Set servo", methods=["PUT"]
        )
        self.add_api_route("/set_aux", self.set_aux, name="Set AUX", methods=["PUT"])
        self.add_api_route(
            "/set_backlight", self.set_backlight, name="Set backlight", methods=["PUT"]
        )
        self.add_api_route(
            "/set_heater", self.set_heater, name="Set heater", methods=["PUT"]
        )
        self.add_api_route("/set_pump", self.set_pump, name="Set pump", methods=["PUT"])

    def set_pump(self, slot: int, value: bool):
        arg0 = slot
        arg1 = 0xFF if value else 0x0
        frame = UInt8MultiArray(data=[0xC0, 0x02, arg0, arg1])
        self.ros2uart_pub.publish(frame)
        return True

    def set_heater(self, slot: int, value: bool):
        arg0 = slot
        arg1 = 0xFF if value else 0x0
        frame = UInt8MultiArray(data=[0xC1, 0x02, arg0, arg1])
        self.ros2uart_pub.publish(frame)
        return True

    def set_backlight(self, slot: int, value: bool):
        arg0 = slot
        arg1 = 0xFF if value else 0x0
        frame = UInt8MultiArray(data=[0xC2, 0x02, arg0, arg1])
        self.ros2uart_pub.publish(frame)
        return True

    def set_aux(self, slot: int, value: bool):
        arg0 = slot
        arg1 = 0xFF if value else 0x0
        frame = UInt8MultiArray(data=[0xC3, 0x02, arg0, arg1])
        self.ros2uart_pub.publish(frame)
        return True

    def set_servo(self, slot: int, angle_deg: int):
        if angle_deg < 0 or angle_deg > 180:
            self.parent_node.get_logger().error(
                "Service did not process request: bad servo angle"
            )
            return False

        arg0 = slot
        arg1 = angle_deg
        frame = UInt8MultiArray(data=[0xC4, 0x02, arg0, arg1])
        self.ros2uart_pub.publish(frame)
        return True

    def set_carousel(self, value: int):
        frame = UInt8MultiArray(
            data=[0x71, 0x06, 0x01, 0x02, value, value, value, value]
        )

        self.ros2uart_pub.publish(frame)
        return True

    def open_close_sample(self, open: bool):
        value = 50 if open else 100

        frame = UInt8MultiArray(
            data=[0x71, 0x06, 0x01, 0x01, value, value, value, value]
        )

        self.ros2uart_pub.publish(frame)
        return True

    def get_weight(self):
        frame = UInt8MultiArray(data=[0x74, 0x01, 0x01])
        self.ros2uart_pub.publish(frame)
        return True

    def tare_weight(self):
        r = rospkg.RosPack()
        pkg_path = r.get_path("kalman_groundstation")
        current_cfg_path: str = os.path.join(
            pkg_path, f"cfg/science/current_weight.yaml"
        )
        tared_cfg_path: str = os.path.join(pkg_path, f"cfg/science/tared_weight.yaml")

        weight = None
        with open(current_cfg_path, "r") as stream:
            weight = yaml.safe_load(stream)["weight"]

        with open(tared_cfg_path, "w") as stream:
            yaml.safe_dump({"weight": weight}, stream)
        return True

    def get_smart_probe(self):
        frame = UInt8MultiArray(data=[0xFD, 0])
        self.ros2uart_pub.publish(frame)
        return True

    def get_panorama(self, lat: float, lon: float, alt: float):
        msg = Vector3(lat, lon, alt)
        self.gps_panorama_logger.log(msg)

        self.panorama_trigger_publisher.publish(Empty())
        return True

    def log_user_marker(self, lat: float, lon: float, alt: float, desc: str):
        msg = Vector3(lat, lon, alt)
        self.gps_user_marker_logger.log(msg, desc)
        return True

    def lamp_pwm(self, value: int):
        frame = UInt8MultiArray(data=[0x72, 8, 1, 8, 0, 0, 0, value, 0, 0])
        self.ros2uart_pub.publish(frame)
        return True
