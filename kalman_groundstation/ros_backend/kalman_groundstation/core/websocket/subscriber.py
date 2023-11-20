import rclpy
from rclpy.node import Node
import yaml

from .message import Message


class Subscriber:
    def __init__(self, node: Node, topic: str, type, send) -> None:
        self.node = node
        self.subscriber = node.create_subscription(type, topic, self.callback, 10)
        self.topic = topic
        self.send = send

    def callback(self, message):
        d = self._msg_to_dict(message)
        message = Message(topic=self.topic, data=d)
        self.send(message)

    def _msg_to_dict(self, message) -> dict:
        return yaml.safe_load(str(message))
