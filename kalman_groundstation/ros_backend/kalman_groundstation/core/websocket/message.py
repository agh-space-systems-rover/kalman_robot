from dataclasses import dataclass


@dataclass
class Message:
    """
    Dataclass for ROS messages sent through the websocket.

    Attributes:
    - `topic: str`
    - `data: dict`
    """
    topic: str
    data: dict
