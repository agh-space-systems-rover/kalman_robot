import struct
import rclpy
import os
import yaml
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

BOARD_ID = 0
DEVICE_ID = 0

RAIL_BOARD_ID = 0
RAIL_DEVICE_ID = 1
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
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.master_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.PH_RES)[1:]}",
            self.cb_master_res,
            10,
        )

    def cb_value_req(self, request, response):
        # Create the request message
        req_msg = MasterMessage()
        req_msg.cmd = MasterMessage.PH_REQ
        req_msg.data = struct.pack("BB", BOARD_ID, DEVICE_ID)

        # Publish the request to the master
        self.master_pub.publish(req_msg)
        response.success = True
        response.message = "pH value requested"
        return response

    def cb_rail_control(self, msg: Float32):
        rail_msg = MasterMessage()
        rail_msg.cmd = MasterMessage.PH_RAIL
        target_vel = max(-1.0, min(1.0, msg.data))
        target_vel_int = int(target_vel * RAIL_MAX_SPEED)
        rail_msg.data = struct.pack("BBB", RAIL_BOARD_ID, RAIL_DEVICE_ID, target_vel_int)
        self.master_pub.publish(rail_msg)

    def cb_master_res(self, msg: MasterMessage):
        if len(msg.data) < 4:
            self.get_logger().warn("Received invalid pH response")
            return

        # Unpack the response
        board_id, device_id, ph_value = struct.unpack("<BBH", msg.data[:4])

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

        # Publish the value if IDs match
        if board_id == BOARD_ID and device_id == DEVICE_ID:
            self.value_pub.publish(Float32(data=float(ph_value)))
        else:
            self.get_logger().warn(
                f"Unknown pH sensor response: board_id={board_id}, device_id={device_id}"
            )

def main():
    try:
        rclpy.init()
        node = PHDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
