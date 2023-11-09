import importlib
from io import BytesIO
from itertools import chain
import json
import os
import time
from typing import Any, Dict, List, Tuple
import genpy
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import UInt8MultiArray

from kalman_rosou.utils import (
    FrameDirection,
    Config,
    Uint8,
    create_header,
)


class BigAutonomyTranslator(Node):
    def __init__(self):
        super().__init__("Rosou")  # type: ignore
        self.ros2uart_publisher = self.create_publisher(
            UInt8MultiArray, "ros2uart", 10
        )

        self.declare_parameter("is_station", False)

        self.publishers_: Dict[int, Publisher] = {}
        self.subscribers_: Dict[int, Subscription] = {}
        self.configs: Dict[int, Config] = {}
        self.timers_: Dict[int, float] = {}

        self._load_configs()
        self._import_messages()

        config: Config
        if self.is_station():
            self.create_subscription(
                UInt8MultiArray,
                "uart2ros/_129",
                self.receive,
                10
            )
            max_idx: int = 0
            # setup station publishers
            for idx, config in enumerate(self.configs_rover, start=max_idx):
                self.publishers_[idx] = self.create_publisher(
                    self._get_message_type(config), config["topic"], 10
                )
                max_idx = idx
                self.configs[idx] = config

            # setup station subscribers
            for idx, config in enumerate(self.configs_station, start=max_idx + 1):
                callback = lambda msg: self.send(
                    FrameDirection.TO_UART, msg, config, idx
                )
                self.subscribers_[idx] = self.create_subscription(
                    self._get_message_type(config), config["topic"], callback, 10
                )
                self.timers_[idx] = 0.0
                self.configs[idx] = config
                max_idx = idx

        else:  # rover
            self.create_subscription(
                UInt8MultiArray, "uart2ros/_128", lambda msg: self.receive(msg), 10
            )

            max_idx = 0

            # mind the order of publishers and subscribers.
            # framed IDs send form station must match frame IDs received at rover

            # setup rover subscribers
            for idx, config in enumerate(self.configs_rover, start=max_idx):
                callback = lambda msg: self.send(
                    FrameDirection.TO_RF, msg, config, idx
                )
                self.subscribers_[idx] = self.create_subscription(
                    self._get_message_type(config), config["topic"], callback, 10
                )
                self.timers_[idx] = 0
                self.configs[idx] = config
                max_idx = idx

            # setup rover publishers
            for idx, config in enumerate(self.configs_station, start=max_idx + 1):
                self.publishers_[idx] = self.create_publisher(
                    self._get_message_type(config), config["topic"], 10
                )
                self.configs[idx] = config
                max_idx = idx

        self._log_publisher_subscriber_dicts()

    def _log_publisher_subscriber_dicts(self):
        log_path: str = os.path.join(
            get_package_share_directory("kalman_rosou"),
            "log/pub_sub.json"
        )

        with open(log_path, "w") as log_file:
            temp = {
                f"{idx}-publisher": publisher.topic_name
                for idx, publisher in self.publishers_.items()
            }
            temp.update(
                {
                    f"{idx}-subscriber": subscriber.topic_name
                    for idx, subscriber in self.subscribers_.items()
                }
            )
            json.dump(temp, log_file, ensure_ascii=False, indent=4)

    def _load_configs(self) -> None:
        self.configs_station: List[Config]
        self.configs_rover: List[Config] = []
        config_path = os.path.join(
            get_package_share_directory("kalman_rosou"),
            "config/rosou_config.json"
        )
        try:
            with open(config_path) as file:
                configs_all = json.load(file)
                self.configs_station = configs_all["station"]
                self.configs_rover = configs_all["rover"]
            self.get_logger().info("Config loaded correctly")
        except Exception as e:
            self.get_logger().error("Failed to load config error {}".format(e))
            raise e

    def _import_messages(self) -> None:
        self.message_modules = {}
        for config in chain(self.configs_rover, self.configs_station):
            module_name: str = config["message_type"].split("/")[0]
            if module_name not in self.message_modules:
                self.message_modules[module_name] = [
                    importlib.import_module(module_name + ".msg")
                ]

    def _get_message_type(self, config: Config) -> Any:
        module_name: str = config["message_type"].split("/")[0]
        msg_type_name: str = config["message_type"].split("/")[1]
        for module in self.message_modules[module_name]:
            if hasattr(module, msg_type_name):
                return getattr(module, msg_type_name)

    def receive(self, msg: UInt8MultiArray) -> None:
        frame_id: int = msg.data[2]
        data: bytes = msg.data[3:]

        # NON BAT CUSTOM FRAMES SHOULD YOU ID >= 100
        if frame_id >= 100:
            return

        config = self.configs[frame_id]

        msg = self._get_message_type(config)()
        eval(config["message_part_serialized"] + ".deserialize(data)")

        self.publishers_[frame_id].publish(msg)

    def send(
        self,
        frame_direction: FrameDirection,
        msg: genpy.Message,
        config: Config,
        frame_id: int,
    ) -> None:
        # `msg` argument in used in eval

        period: float = config["max_frequency"]
        now: float = time.time()

        if (now - self.timers_[frame_id]) < period:
            # we don't send anything after time shorter than period
            return

        self.timers_[frame_id] = now

        message_part_serialized = eval(config["message_part_serialized"])

        data = BytesIO()
        message_part_serialized.serialize(data)

        frame_header: Tuple[Uint8, Uint8, Uint8] = create_header(
            frame_direction, frame_id, data.getvalue()
        )

        frame = UInt8MultiArray()
        frame.data = frame_header + tuple(bytearray(data.getvalue()))

        self.ros2uart_publisher.publish(frame)

    def is_station(self) -> bool:
        return self.get_parameter("is_station").get_parameter_value().bool_value


def main(args=None):
    rclpy.init(args=args)

    bat = BigAutonomyTranslator()

    rclpy.spin(bat)
    bat.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
