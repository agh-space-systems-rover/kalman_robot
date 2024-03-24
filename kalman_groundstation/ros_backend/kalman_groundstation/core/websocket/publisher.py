import rclpy
from rclpy.node import Node
from .message import Message


class Publisher:
    """
    Publisher wrapper for ROS2 -> websocket communication. Basically, it is a publisher of Message 
    on specified `topic` of specified `type`.

    Example:
    ```python
    publisher = Publisher(node, "/topic", String)
    publisher.publish(Message(topic="/topic", data="hello"))
    ```
    """
    def __init__(self, node: Node, topic: str, type) -> None:
        self.node = node
        self.publisher = node.create_publisher(type, topic, 10)
        self.type = type

    def convertToRosMessage(self, data):
        return self.type(**data)

    def publish(self, message: Message):
        try:
            msg = self.convertToRosMessage(message.data)
            self.publisher.publish(msg)
        except AssertionError as e: #
            self.node.get_logger().error(f"[WEBSOCKET] Unable to publish message: {e}")
