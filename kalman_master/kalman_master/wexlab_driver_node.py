import struct
import rclpy
from rclpy.node import Node
import numpy as np
from kalman_interfaces.msg import MasterMessage, WExLabHeaterCfg, WExLabLedAll, WExLabLedSingle, WExLabTemperature
from std_msgs.msg import Float32, Bool, Empty, UInt8


class WExLabDriver(Node):
    def __init__(self):
        super().__init__("wexlab_driver")

        # Configuration for scale reading
        # TODO: Replace with proper calibration parameters!
        self.declare_parameter("weight_scale", 1.0)
        self.declare_parameter("weight_bias", 0.0)

        # Master comms
        self.master_pub = self.create_publisher(MasterMessage, "master_com/ros_to_master", 10)

        self.master_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.ESP_TO_GS)[1:]}",
            self.master_res_cb,
            10,
        )

        self.pump_sub = self.create_subscription(Float32, "wexlab/pump/rate_cmd", self.pump_rate_cb, 10)

        self.heater_toggle_sub = self.create_subscription(Bool, "wexlab/heater/on_off", self.heater_toggle_cb, 10)
        self.heater_cfg_sub = self.create_subscription(WExLabHeaterCfg, "wexlab/heater/cfg", self.heater_cfg_cb, 10)

        self.weight_req_sub = self.create_subscription(Empty, "wexlab/weight/req", self.weight_req_cb, 10)
        self.weight_tare_pub = self.create_subscription(Empty, "wexlab/weight/tare", self.weight_tare_cb, 10)
        self.weight_res_pub = self.create_publisher(Float32, "wexlab/weight/res", 10)

        self.temperature_req_sub = self.create_subscription(UInt8, "wexlab/temperature/req", self.temperature_req_cb, 10)
        self.temperature_res_pub = self.create_publisher(WExLabTemperature, "wexlab/temperature/res", 10)

        self.lid_open_sub = self.create_subscription(Float32, "wexlab/lid/open_cmd", self.lid_open_cb, 10)
        self.lid_toggle_sub = self.create_subscription(Bool, "wexlab/lid/on_off", self.lid_toggle_cb, 10)

        self.led_all_sub = self.create_subscription(WExLabLedAll, "wexlab/led/all", self.led_all_cb, 10)
        self.led_single_sub = self.create_subscription(WExLabLedSingle, "wexlab/led/single", self.led_single_cb, 10)

        # Track lid state to reconstruct commands
        self.lid_is_on = False
        self.lid_open_pct = 0.0

    def pump_rate_cb(self, msg: Float32):
        pwm = int(np.clip(msg.data, 0, 100))
        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP
        out_msg.data = [0x10, 2, 1, pwm]
        self.master_pub.publish(out_msg)

    def heater_toggle_cb(self, msg: Bool):
        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP
        out_msg.data = [0x11, 1, 1 if msg.data else 0]
        self.master_pub.publish(out_msg)

    def heater_cfg_cb(self, msg: WExLabHeaterCfg):
        high_temp = int(np.clip(msg.thermostat_max, 0, 255))
        low_temp = int(np.clip(msg.thermostat_min, 0, 255))
        pwm_main = int(np.clip(msg.power_main, 0, 100))
        pwm_lid = int(np.clip(msg.power_lid, 0, 100))

        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP
        out_msg.data = [0x12, 4, high_temp, low_temp, pwm_main, pwm_lid]
        self.master_pub.publish(out_msg)

    def weight_req_cb(self, msg: Empty):
        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP
        out_msg.data = [0xD0, 0]
        self.master_pub.publish(out_msg)

    def weight_tare_cb(self, msg: Empty):
        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP
        out_msg.data = [0xD2, 0]
        self.master_pub.publish(out_msg)

    def temperature_req_cb(self, msg: UInt8):
        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP
        out_msg.data = [0x15, 1, int(msg.data)]
        self.master_pub.publish(out_msg)

    def master_res_cb(self, msg: MasterMessage):
        if len(msg.data) == 0:
            return

        cmd = msg.data[0]
        if cmd == 0xD1:
            if len(msg.data) < 6:
                self.get_logger().warn("Received invalid weight response length")
                return

            packed_data = bytes(msg.data[2:6])
            voltage_mv = struct.unpack("<i", packed_data)[0]

            self.get_logger().info(f"data = {list(msg.data)}")
            self.get_logger().info(f"data = {list(msg.data[2:6])}")
            self.get_logger().info(f"int = {struct.unpack('<i', packed_data)[0]}")

            # Convert to float
            voltage_mv_f = float(voltage_mv)

            # Read calibration params
            scale = self.get_parameter("weight_scale").value
            bias = self.get_parameter("weight_bias").value

            # Calculate weight in grams
            weight_g = voltage_mv_f * scale + bias

            self.weight_res_pub.publish(Float32(data=weight_g))

        if cmd == 0x16:
            if len(msg.data) < 6:
                self.get_logger().warn("Received invalid temperature response length")
                return

            temperature_id = msg.data[2]
            temperature_error = msg.data[3] > 0

            packed_temp = bytes(msg.data[4:6])
            temperature = struct.unpack("<h", packed_temp)[0] / 100.0

            self.temperature_res_pub.publish(WExLabTemperature(
                temperature_id=temperature_id,
                temperature=temperature,
                temperature_error=temperature_error
            ))

    def update_lid_servo(self):
        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP

        if not self.lid_is_on:
            # Over 100 turns off the servo
            out_msg.data = [0x13, 1, 255]
        else:
            pwm = int(np.clip(self.lid_open_pct, 0, 100))
            out_msg.data = [0x13, 1, pwm]

        self.master_pub.publish(out_msg)

    def lid_open_cb(self, msg: Float32):
        self.lid_open_pct = msg.data
        if self.lid_is_on:
            self.update_lid_servo()

    def lid_toggle_cb(self, msg: Bool):
        self.lid_is_on = msg.data
        self.update_lid_servo()

    def led_all_cb(self, msg: WExLabLedAll):
        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP
        out_msg.data = [
            0x20,
            3,
            int(msg.color.r * 255),
            int(msg.color.g * 255),
            int(msg.color.b * 255)
        ]
        self.master_pub.publish(out_msg)

    def led_single_cb(self, msg: WExLabLedSingle):
        out_msg = MasterMessage()
        out_msg.cmd = MasterMessage.GS_TO_ESP
        out_msg.data = [
            0x21,
            4,
            int(msg.led_id),
            int(msg.color.r * 255),
            int(msg.color.g * 255),
            int(msg.color.b * 255)
        ]
        self.master_pub.publish(out_msg)


def main():
    try:
        rclpy.init()
        node = WExLabDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
