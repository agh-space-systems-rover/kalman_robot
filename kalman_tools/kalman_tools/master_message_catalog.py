from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Iterable

from kalman_interfaces.msg import MasterMessage

HEX_CMD_RE = re.compile(r"^0?x?[0-9a-fA-F]{1,2}$")
DEC_CMD_RE = re.compile(r"^\d{1,3}$")


@dataclass(frozen=True)
class FrameDefinition:
    name: str
    cmd: int
    hex_label: str

    @property
    def label(self) -> str:
        return f"{self.name} ({self.hex_label})"

    @property
    def receive_topic(self) -> str:
        return cmd_to_receive_topic(self.cmd)


def cmd_to_receive_topic(cmd: int) -> str:
    return f"master_com/master_to_ros/{hex(cmd)[1:]}"


def list_frame_definitions() -> list[FrameDefinition]:
    frames: list[FrameDefinition] = []
    for name in dir(MasterMessage):
        if name.startswith("_"):
            continue
        value = getattr(MasterMessage, name)
        if not isinstance(value, int):
            continue
        frames.append(
            FrameDefinition(
                name=name,
                cmd=value,
                hex_label=hex(value),
            )
        )
    frames.sort(key=lambda frame: (frame.cmd, frame.name))
    return frames


def frame_by_name(name: str) -> FrameDefinition | None:
    for frame in list_frame_definitions():
        if frame.name == name:
            return frame
    return None


def frames_for_cmd(cmd: int) -> list[FrameDefinition]:
    return [frame for frame in list_frame_definitions() if frame.cmd == cmd]


def format_cmd(cmd: int) -> str:
    return hex(cmd)


def parse_payload(text: str) -> list[int]:
    stripped = text.strip()
    if not stripped:
        return []

    parts = re.split(r"[\s,;]+", stripped)
    values: list[int] = []
    for part in parts:
        if not part:
            continue
        if part.lower().startswith("0x"):
            value = int(part, 16)
        elif HEX_CMD_RE.match(part) and any(ch in "abcdefABCDEFxX" for ch in part):
            value = int(part, 16)
        else:
            value = int(part, 10)
        if value < 0 or value > 255:
            raise ValueError(f"Payload byte out of range (0-255): {value}")
        values.append(value)
    return values


def parse_cmd(text: str) -> int:
    stripped = text.strip()
    if not stripped:
        raise ValueError("Command is empty")

    for frame in list_frame_definitions():
        if stripped == frame.name:
            return frame.cmd

    if stripped.lower().startswith("0x"):
        value = int(stripped, 16)
    elif HEX_CMD_RE.match(stripped) and any(ch in "abcdefABCDEFxX" for ch in stripped):
        value = int(stripped, 16)
    elif DEC_CMD_RE.match(stripped):
        value = int(stripped, 10)
    else:
        raise ValueError(f"Unknown command: {stripped}")

    if value < 0 or value > 255:
        raise ValueError(f"Command out of range (0-255): {value}")
    return value


def format_payload(data: Iterable[int]) -> str:
    return ", ".join(str(byte) for byte in data)


def format_payload_hex(data: Iterable[int]) -> str:
    return " ".join(f"0x{byte:02x}" for byte in data)
