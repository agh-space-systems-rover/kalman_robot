import struct
import rclpy
import os
import yaml
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage
from std_msgs.msg import Float32, UInt8MultiArray
from std_srvs.srv import Trigger

BOARD_ID = 0
CHANNEL_ID = 0

RAIL_BOARD_ID = 0
RAIL_CHANNEL_ID = 1
RAIL_MAX_SPEED = 100

class PHDriver(Node):
    def __init__(self):
        super().__init__("ph_driver")

        # Publisher for the single pH sensor
        self.value_pub = self.create_publisher(Float32, "science/ph/value", 10)
        self.value_raw_pub = self.create_publisher(Float32, "science/ph/value/raw", 10)

        self.value_req_srv = self.create_service(
            Trigger,
            "science/ph/value/req",
            self.cb_value_req,
        )

        self.rail_control_sub = self.create_subscription(
            Float32,
            "science/ph/rail/target_vel",
            self.cb_rail_control,
            10
        )

        # Master comms
        self.requester = self.create_publisher(
            UInt8MultiArray, "/kutong/request", 10
        )
        self.data_response = self.create_subscription(
            UInt8MultiArray,
            "kutong/data",
            self.cb_data_response,
            10,
        )

    def cb_value_req(self, request, response):
        # Publish the request to the master
        self.requester.publish(UInt8MultiArray())
        response.success = True
        response.message = "pH value requested"
        return response

    def cb_rail_control(self, msg: Float32):
        target_vel = max(-1.0, min(1.0, msg.data))
        target_vel_int = int(abs(target_vel * RAIL_MAX_SPEED))
        
        rail_msg = MasterMessage()
        rail_msg.cmd = MasterMessage.PH_RAIL
        rail_msg.data = [RAIL_BOARD_ID, RAIL_CHANNEL_ID, target_vel_int, 0 if target_vel < 0 else 1]
        # self.master_pub.publish(rail_msg)

    def cb_data_response(self, msg: UInt8MultiArray):
        if len(msg.data) < 4:
            self.get_logger().warn("Received invalid pH response")
            return

        # Unpack the response
        ph_value, _ = struct.unpack("<HH", msg.data)
        # self.get_logger().info(f"{v1} {ph_value}")

        # Publish the raw value
        self.value_raw_pub.publish(Float32(data=float(ph_value)))

        # Load calibration from ~/.config/kalman/ph_calib.yaml
        path = os.path.expanduser("~/.config/kalman/ph_calib.yaml")
        if os.path.exists(path):
            with open(path, "r") as f:
                calib_data = yaml.safe_load(f)

            # Apply calibration if available
            if "scale" in calib_data and "bias" in calib_data:
                ph_value = (ph_value * calib_data["scale"]) + calib_data["bias"]
            else:
                self.get_logger().warn("Calibration data missing in ph_calib.yaml")
        else:
            self.get_logger().warn(f"Calibration file not found: {path}")

        self.value_pub.publish(Float32(data=float(ph_value)))

def main():
    try:
        rclpy.init()
        node = PHDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
