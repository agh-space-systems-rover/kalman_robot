import copy
import os
import struct

import rclpy
import yaml
from rclpy.node import Node
from kalman_interfaces.msg import MasterMessage
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

STORAGE_CONFIG_PATH = os.path.expanduser("~/.config/kalman/storage_calib.yaml")

STORAGE_DEFAULT_CONFIG = {
    "sand": [
        {
            "board": 0,
            "channel": 3,
            # calibration coeffs
            "scale": 23739.0 / 1310000.0,
            "bias": -44568183.0 / 1310000.0,
            # safe angles for the servo
            "open_angle": 176,
            "close_angle": 0,
        }
    ],
    "rock": [
        {
            "board": 0,
            "channel": 0,
            "scale": (23739.0 / 1425700.0) * 1.05,
            "bias": -447639993.0 / 712850.0,
            "open_angle": 192,
            "close_angle": 0,
        },
        {
            "board": 0,
            "channel": 2,
            "scale": (23739.0 / 1425700.0) * 1.05,
            "bias": 0.0,
            "open_angle": 192,
            "close_angle": 0,
        },
    ],
}


def load_storage_config():
    storage_config = copy.deepcopy(STORAGE_DEFAULT_CONFIG)
    if not os.path.exists(STORAGE_CONFIG_PATH):
        return storage_config

    with open(STORAGE_CONFIG_PATH, "r") as f:
        storage_calib = yaml.safe_load(f)

    storage_by_address = {
        (info["board"], info["channel"]): info
        for infos in storage_config.values()
        for info in infos
    }
    for calibration in storage_calib:
        storage_info = storage_by_address[
            (calibration["board"], calibration["channel"])
        ]
        storage_info["scale"] = calibration["scale"]
        storage_info["bias"] = calibration["bias"]
    return storage_config


class StorageDriver(Node):
    def __init__(self):
        super().__init__("storage_driver")

        self.storage_config = load_storage_config()

        # Create publishers for each storage
        self.api_pubs = {}
        for storage_name in self.storage_config.keys():
            val_topic = f"science/storage/{storage_name}/weight"
            self.api_pubs[storage_name] = self.create_publisher(Float32, val_topic, 10)

        # Create value request services for each storage
        self.value_req_srv = {}
        for storage_name in self.storage_config.keys():
            req_srv_name = f"science/storage/{storage_name}/weight/req"
            self.value_req_srv[storage_name] = self.create_service(
                Trigger,
                req_srv_name,
                lambda req, res, name=storage_name: self.handle_value_req(name, res),
            )

        # Master comms
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.master_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.SCALE_RES)[1:]}",
            self.cb_master_res,
            10,
        )

        # Create open/close services for each storage
        self.open_srv = {}
        self.close_srv = {}
        for storage_name in self.storage_config.keys():
            open_srv_name = f"science/storage/{storage_name}/open"
            close_srv_name = f"science/storage/{storage_name}/close"
            self.open_srv[storage_name] = self.create_service(
                Trigger,
                open_srv_name,
                lambda req, resp, name=storage_name: self.handle_servo(
                    name, True, resp
                ),
            )
            self.close_srv[storage_name] = self.create_service(
                Trigger,
                close_srv_name,
                lambda req, resp, name=storage_name: self.handle_servo(
                    name, False, resp
                ),
            )

        # Track last values to sum them up for multi-channel storages
        self.last_values = {
            name: [0.0] * len(infos)
            for name, infos in self.storage_config.items()
        }

    def handle_value_req(self, storage_name, response):
        storage_infos = self.storage_config[storage_name]
        for storage_info in storage_infos:
            board_id = storage_info["board"]
            channel_id = storage_info["channel"]

            # Create the request message
            req_msg = MasterMessage()
            req_msg.cmd = MasterMessage.SCALE_REQ
            req_msg.data = struct.pack("BB", board_id, channel_id)

            # Publish the request to the master
            self.master_pub.publish(req_msg)

        response.success = True
        response.message = f"Requested value for {storage_name}"
        return response

    def cb_master_res(self, msg: MasterMessage):
        if len(msg.data) < 6:
            self.get_logger().warn("Received invalid scale response")
            return

        padded_data = bytes(msg.data[:6])
        board_id, channel_id, value = struct.unpack("<BBi", padded_data)

        self.get_logger().info(f"{channel_id} = {value}")
        self.storage_config = load_storage_config()

        # Find which storage this corresponds to
        storage_name = None
        info_idx = 0
        for name, infos in self.storage_config.items():
            for idx, info in enumerate(infos):
                if info["board"] == board_id and info["channel"] == channel_id:
                    storage_name = name
                    info_idx = idx
                    break
            if storage_name:
                break

        # Publish the value
        if storage_name:
            # Apply linear mapping
            value = (
                value * self.storage_config[storage_name][info_idx]["scale"]
                + self.storage_config[storage_name][info_idx]["bias"]
            )
            self.last_values[storage_name][info_idx] = value
            total_value = sum(self.last_values[storage_name])

            self.get_logger().info(f"{self.last_values}")
            self.api_pubs[storage_name].publish(Float32(data=float(total_value)))
        else:
            self.get_logger().warn(
                f"Unknown storage response: board_id={board_id}, channel_id={channel_id}"
            )

    def handle_servo(self, storage_name, open_flag, response):
        storage_infos = self.storage_config[storage_name]
        for storage_info in storage_infos:
            board_id = storage_info["board"]
            channel_id = storage_info["channel"]
            angle = (
                storage_info["open_angle"] if open_flag else storage_info["close_angle"]
            )

            msg = MasterMessage()
            msg.cmd = MasterMessage.SERVO_SET
            msg.data = struct.pack("BBB", board_id, channel_id, angle)
            self.master_pub.publish(msg)

        response.success = True
        response.message = (
            f"{'Opened' if open_flag else 'Closed'} {storage_name} (angle={angle})"
        )
        return response


def main():
    try:
        rclpy.init()
        node = StorageDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
