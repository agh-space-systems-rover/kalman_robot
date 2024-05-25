import rclpy
from rclpy.node import Node
from rclpy_message_converter.message_converter import convert_ros_message_to_dictionary

from .message import Message


class Subscriber:
    """
    Subscriber wrapper for ROS2 -> websocket communication. Basically, it subscribes to a topic and
    sends the message to the websocket server (or any other function passed as `send`).

    Sends a Message object to the websocket server.

    Example:
    ```python
    def send(message: Message):
        # do something with the message
        pass
        
    subscriber = Subscriber(node, "/topic", String, send)
    ```
    """
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
        return convert_ros_message_to_dictionary(message)
