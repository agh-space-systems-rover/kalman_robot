import rclpy
from rclpy.node import Node
from .message import Message

def force_correct_types(data: dict, target_type: type) -> dict:
    ## TODO: extend or use a dedicated library
    type_conversions = {
        'double': float,
        'int64': int,
    }
    fields_and_field_types = target_type.get_fields_and_field_types()

    ret = {}
    for key in fields_and_field_types:
        current_type = fields_and_field_types[key]
        value = data[key]
        if value is None:
            continue

        if current_type in type_conversions:
            conv = type_conversions[current_type]
            ret[key] = conv(value)
        else:
            ret[key] = value

    return ret



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
        data = force_correct_types(data, self.type)
        return self.type(**data)

    def publish(self, message: Message):
        try:
            msg = self.convertToRosMessage(message.data)
            self.publisher.publish(msg)
        except AssertionError as e: #
            self.node.get_logger().error(f"[WEBSOCKET] Unable to publish message: {e}")
