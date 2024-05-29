import os
import struct

import rospkg
import yaml
from fastapi import APIRouter, Path
from geometry_msgs.msg import Vector3
from kalman_groundstation.modules.science.universal_module import (
    CAN_CMD_INPUT_REQUEST,
    CAN_CMD_SET_DIGITAL_OUTPUT,
    CAN_CMD_SET_HBRIDGE,
    CAN_CMD_SET_LED_DRIVER,
    CAN_CMD_SET_PWM_OUTPUT,
    CAN_CMD_SET_STEPPER_POSITION,
    CAN_CMD_STEPPER_HOMING_REQUEST,
    CAN_CMD_STEPPER_POSITION_REQUEST,
    CAN_CMD_WEIGHT_REQUEST,
    CAN_CMD_AUTOMATION_SEQUENCE_BEGIN_REQUEST,
    CAN_CMD_AUTOMATION_SEQUENCE_STATE_REQUEST,
    SequenceManagerFrame_Format,
    HBridgeFrame_Format,
    PWMFrame_Format,
    RequestFrame_Format,
    StepperFrame_Format,
    PWMFrame,
    HBridgeFrame,
    create_HBridgeFrame,
    create_PWMFrame,
    create_StepperFrame,
    create_RequestFrame,
    create_SequenceManagerFrame,
)
from kalman_groundstation.utils.logger import GpsLogger
from kalman_interfaces.msg import MasterMessage
from rclpy.node import Node
from std_msgs.msg import Empty


class ScienceRouter(APIRouter):
    def __init__(self, parent_node: Node) -> None:
        super().__init__(prefix="", tags=["science"])
        self.parent_node = parent_node

        self.ros2uart_pub = parent_node.create_publisher(
            MasterMessage, "/master_com/ros_to_master", qos_profile=10
        )

        self.panorama_trigger_publisher = parent_node.create_publisher(
            Empty, "/panorama_trigger", qos_profile=10
        )

        self.gps_panorama_logger = GpsLogger("gps_panorama")
        self.gps_user_marker_logger = GpsLogger("gps_user_marker")

        self.add_api_route(
            "/raw_set_digital_output",
            self.raw_set_digital_output,
            name="Use CAN to set digital output",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_set_pwm_output",
            self.raw_set_pwm_output,
            name="Use CAN to set pwm output",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_set_led_driver",
            self.raw_set_led_driver,
            name="Use CAN to set led driver output",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_set_hbridge",
            self.raw_set_hbridge,
            name="Use CAN to set H-bridge output",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_set_stepper_position",
            self.raw_set_stepper_position,
            name="Use CAN to set stepper position",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_stepper_homing_request",
            self.raw_stepper_homing_request,
            name="Use CAN to send stepper homing request",
            methods=["PUT"],
        )

        self.add_api_route(
            "/set_h_bridge",
            self.set_h_bridge,
            name="Set value for h_bridge with specified channel",
            methods=["PUT"],
        )

        self.add_api_route(
            "/led_state",
            self.led_state,
            name="Set LED state on selected channel",
            methods=["PUT"],
        )

        self.add_api_route(
            "/sequence_begin",
            self.sequence_begin,
            name="Request start sequence corresponding to id",
            methods=["PUT"],
        )

        self.add_api_route(
            "/sequence_state",
            self.sequence_state,
            name="Request sequence state corresponding to id",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_weight_request",
            self.raw_weight_request,
            name="Use CAN to send stepper homing request",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_input_request",
            self.raw_input_request,
            name="Use CAN to send analog input request",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_stepper_position_request",
            self.raw_stepper_position_request,
            name="Use CAN to send stepper position request",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_sequence_begin",
            self.raw_sequence_begin,
            name="Use CAN to send sequence begin request",
            methods=["PUT"],
        )

        self.add_api_route(
            "/raw_sequence_state_req",
            self.raw_sequence_state_req,
            name="Use CAN to send sequence state request",
            methods=["PUT"],
        )

        self.add_api_route(
            "/lamp_pwm",
            self.lamp_pwm,
            name="Set PWM for turning on lamp",
            methods=["PUT"],
        )
        self.add_api_route(
            "/set_pwm",
            self.set_pwm,
            name="Set pwm on selected channel",
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
            "/set_carousel_with_offset", self.set_carousel_with_offset, name="Set carousel with offset", methods=["PUT"]
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

    def raw_set_digital_output(self, board_id: int, channel_id: int, value: int):
        assert 0 <= value and value <= 145
        msg = MasterMessage()
        msg.cmd = CAN_CMD_SET_DIGITAL_OUTPUT
        msg.data = create_PWMFrame(board_id, channel_id, value).pack()
        self.ros2uart_pub.publish(msg)

    def raw_set_pwm_output(self, board_id: int, channel_id: int, value: int):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_SET_PWM_OUTPUT
        msg.data = create_PWMFrame(board_id, channel_id, value).pack()
        self.ros2uart_pub.publish(msg)

    def raw_set_led_driver(self, board_id: int, channel_id: int, value: int):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_SET_LED_DRIVER
        msg.data = create_PWMFrame(board_id, channel_id, value).pack()
        self.ros2uart_pub.publish(msg)

    def raw_set_hbridge(
        self, board_id: int, channel_id: int, speed: int, direction: int
    ):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_SET_HBRIDGE
        msg.data = create_HBridgeFrame(board_id, channel_id, speed, direction).pack()
        self.ros2uart_pub.publish(msg)

    def raw_set_stepper_position(
        self, board_id: int, channel_id: int, target_position: float
    ):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_SET_STEPPER_POSITION
        msg.data = create_StepperFrame(board_id, channel_id, target_position).pack()
        self.ros2uart_pub.publish(msg)

    def raw_stepper_homing_request(self, board_id: int, channel_id: int):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_STEPPER_HOMING_REQUEST
        msg.data = create_RequestFrame(board_id, channel_id).pack()
        self.ros2uart_pub.publish(msg)

    def raw_weight_request(self, board_id: int, channel_id: int):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_WEIGHT_REQUEST
        msg.data = create_RequestFrame(board_id, channel_id).pack()
        self.ros2uart_pub.publish(msg)

    def raw_input_request(self, board_id: int, channel_id: int):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_INPUT_REQUEST
        msg.data = create_RequestFrame(board_id, channel_id).pack()
        self.ros2uart_pub.publish(msg)

    def raw_stepper_position_request(self, board_id: int, channel_id: int):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_STEPPER_POSITION_REQUEST
        msg.data = create_RequestFrame(board_id, channel_id).pack()
        self.ros2uart_pub.publish(msg)

    def raw_sequence_begin(self, board_id: int, sequence_id: int):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_AUTOMATION_SEQUENCE_BEGIN_REQUEST
        msg.data = create_SequenceManagerFrame(board_id, sequence_id).pack()
        self.ros2uart_pub.publish(msg)

    def raw_sequence_state_req(self, board_id: int, sequence_id: int):
        msg = MasterMessage()
        msg.cmd = CAN_CMD_AUTOMATION_SEQUENCE_STATE_REQUEST
        msg.data = create_SequenceManagerFrame(board_id, sequence_id).pack()
        self.ros2uart_pub.publish(msg)

    def set_h_bridge(self, channel: int, speed: int, direction: int):
        assert(0 <= direction and direction <= 1)
        self.raw_set_hbridge(0, channel, speed, direction)
        return True

    def led_state(self, channel: int, value: int):
        self.raw_set_led_driver(0, channel, value)
        return True
    
    def sequence_begin(self, sequence_id: int):
        self.raw_sequence_begin(0, sequence_id)
        return True

    def sequence_state(self, sequence_id: int):
        self.raw_sequence_state_req(0, sequence_id)
        return True
    
    def set_pump(self, slot: int, value: bool):
        self.parent_node.get_logger().error("set_pump: implement me")
        return True

    def set_heater(self, slot: int, value: bool):
        self.parent_node.get_logger().error("set_heater: implement me")
        return True

    def set_backlight(self, slot: int, value: bool):
        self.parent_node.get_logger().error("set_backlight: implement me")
        return True

    def set_aux(self, slot: int, value: bool):
        self.parent_node.get_logger().error("set_aux: implement me")
        return True

    def set_servo(self, slot: int, angle_deg: int):
        self.parent_node.get_logger().error("set_servo: implement me")
        return True

    def set_carousel(self, value: int):
        self.parent_node.get_logger().error("set_carousel: implement me")
        return True

    def set_pwm(self, channel: int, value: int):
        self.raw_set_pwm_output(0, channel_id=channel, value=value)
        return True

    def set_carousel_with_offset(self, value: int, offset: int):
        position = (value * 360.0 / 16.0) + offset
        self.raw_set_stepper_position(0, 0, position)
        return True

    def open_close_sample(self, open: bool):
        self.parent_node.get_logger().error("open_close_sample: implement me")
        return True

    def get_weight(self):
        self.parent_node.get_logger().error("get_weight: implement me")
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
        self.parent_node.get_logger().error("get_smart_probe: implement me")
        return True

    def get_panorama(self, lat: float, lon: float, alt: float):
        msg = Vector3(x=lat, y=lon, z=alt)
        self.gps_panorama_logger.log(msg)

        self.panorama_trigger_publisher.publish(Empty())
        return True

    def log_user_marker(self, lat: float, lon: float, alt: float, desc: str):
        msg = Vector3(x=lat, y=lon, z=alt)
        self.gps_user_marker_logger.log(msg, desc)
        return True

    def lamp_pwm(self, value: int):
        self.parent_node.get_logger().error("lamp_pwm: implement me")
        return True
