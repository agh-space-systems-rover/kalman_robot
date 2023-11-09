from typing import TypedDict, Tuple
from enum import IntEnum


Radians = float
Uint8 = int


class FrameDirection(IntEnum):
    TO_UART = 0x80
    TO_RF = 0x81


class Config(TypedDict):
    topic: str
    message_type: str
    message_part_serialized: str
    max_frequency: int


class ServiceConfig(TypedDict):
    name: str
    type: str


def create_header(
    direction: FrameDirection, custom_id: int, data: bytes
) -> Tuple[Uint8, Uint8, Uint8]:
    assert len(data) < 200
    return (direction, len(data) + 1, custom_id)


def isUint8(data: int) -> bool:
    return isinstance(data, int) and data >= 0 and data <= 255
