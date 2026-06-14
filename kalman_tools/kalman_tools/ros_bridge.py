from __future__ import annotations

import atexit
import base64
import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable

from roslibpy import Message, Ros, Topic

from kalman_tools.master_message_catalog import cmd_to_receive_topic

# Configure standard structured logging
logger = logging.getLogger("kalman_tools.ros_bridge")

MASTER_MESSAGE_TYPE = "kalman_interfaces/msg/MasterMessage"
SEND_TOPIC = "master_com/ros_to_master"
MAX_LOG_SIZE = 500


@dataclass(frozen=True)
class Filter:
    index: int
    op: str
    value: int


@dataclass(frozen=True)
class ReceivedMessage:
    time: datetime
    cmd: int
    topic: str
    data: list[int]


class MasterRosBridge:
    def __init__(
        self,
        host: str = "localhost",
        port: int = 3001,
        on_connection_change: Callable[[bool], None] | None = None,
    ) -> None:
        self.host = host
        self.port = port
        self._on_connection_change = on_connection_change

        self._lock = threading.RLock()
        self._closed = False
        self._connected = False

        self._ros: Ros | None = None
        self._publisher: Topic | None = None
        self._subscribers: dict[int, Topic] = {}
        self._subscribed_cmds: set[int] = set()
        self._filters: list[Filter] = []

        self._spam_threads: dict[str, tuple[threading.Event, threading.Thread]] = {}
        self._received: deque[ReceivedMessage] = deque(maxlen=MAX_LOG_SIZE)

        # Separate thread initialization from connection lifecycles
        self._ros_thread = threading.Thread(target=self._run_ros_loop, daemon=True)
        self._ros_thread.start()
        atexit.register(self.close)

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._connected

    def _set_connected(self, connected: bool) -> None:
        with self._lock:
            if self._connected == connected:
                return
            self._connected = connected
        if self._on_connection_change is not None:
            try:
                self._on_connection_change(connected)
            except Exception as e:
                logger.error("Error executing connection state callback: %s", e)

    def _run_ros_loop(self) -> None:
        """Runs the persistent ROS client event loop using safe run_forever hooks."""
        try:
            ros = Ros(host=self.host, port=self.port)

            def on_ready(*_) -> None:
                logger.info("Successfully connected to Rosbridge server at ws://%s:%s", self.host, self.port)
                publisher = Topic(ros, SEND_TOPIC, MASTER_MESSAGE_TYPE)
                publisher.advertise()
                with self._lock:
                    self._ros = ros
                    self._publisher = publisher
                    self._subscribers.clear()
                    self._resubscribe_locked()
                self._set_connected(True)

            def on_close(*_) -> None:
                logger.warning("Rosbridge connection lost or closed.")
                with self._lock:
                    self._publisher = None
                    self._subscribers.clear()
                self._set_connected(False)

            ros.on("ready", on_ready)
            ros.on("close", on_close)

            # Safe blocking loop execution; natively manages internal reconnections
            ros.run_forever()

        except Exception as e:
            logger.critical("Fatal failure in core ROS bridge runner: %s", e, exc_info=True)
        finally:
            with self._lock:
                self._publisher = None
                self._subscribers.clear()
                self._ros = None
            self._set_connected(False)

    def _resubscribe_locked(self) -> None:
        ros = self._ros
        if ros is None:
            return
        for cmd in self._subscribed_cmds:
            self._subscribe_locked(cmd, ros)

    def _subscribe_locked(self, cmd: int, ros: Ros) -> None:
        if cmd in self._subscribers:
            return
        topic_name = cmd_to_receive_topic(cmd)
        topic = Topic(ros, topic_name, MASTER_MESSAGE_TYPE)
        topic.subscribe(
            lambda message, c=cmd, t=topic_name: self._on_message(c, t, message)
        )
        self._subscribers[cmd] = topic

    def _on_message(self, cmd: int, topic: str, message: dict) -> None:

        logger.info(f"Received message: {message}")

        try:
            raw_data = message.get("data", [])

            # If Rosbridge compressed the uint8[] array into a Base64 string:
            if isinstance(raw_data, str):
                data = list(base64.b64decode(raw_data))
            else:
                data = [int(byte) for byte in raw_data]

        except Exception as exc:
            # Safe logging prevents the background websocket thread from failing silently
            import sys
            logger.error(f"[Bridge Decoder Error] Failed parsing topic {topic}: {exc}", file=sys.stderr)
            return

        with self._lock:
            filters = list(self._filters)

        if not self._matches_filters(data, filters):
            return

        received = ReceivedMessage(
            time=datetime.now(timezone.utc),
            cmd=cmd,
            topic=topic,
            data=data,
        )
        with self._lock:
            self._received.append(received)

    @staticmethod
    def _matches_filters(data: list[int], filters: list[Filter]) -> bool:
        if not filters:
            return True
        for filt in filters:
            if filt.index < 0 or filt.index >= len(data):
                return False
            left = data[filt.index]
            right = filt.value
            if filt.op == "==" and left != right: return False
            elif filt.op == "!=" and left == right: return False
            elif filt.op == "<" and not left < right: return False
            elif filt.op == ">" and not left > right: return False
            elif filt.op == "<=" and not left <= right: return False
            elif filt.op == ">=" and not left >= right: return False
        return True

    def send_frame(self, cmd: int, data: list[int]) -> None:
        with self._lock:
            publisher = self._publisher
        if publisher is None:
            raise RuntimeError("Not connected to Rosbridge node.")
        publisher.publish(Message({"cmd": int(cmd), "data": [int(b) for b in data]}))

    def start_spam(self, key: str, cmd: int, data: list[int], interval_s: float) -> None:
        if interval_s <= 0:
            raise ValueError("Spam interval must be a positive float value.")
        self.stop_spam(key)

        stop_event = threading.Event()

        def spam_loop() -> None:
            logger.info("Spam generator thread started for key '%s'", key)
            while not stop_event.wait(interval_s):
                try:
                    self.send_frame(cmd, data)
                except RuntimeError:
                    # Resilient handling: drop warning and keep trying until stop_event triggers
                    logger.warning("Spam key '%s' failed to write frame (link down). Retrying next tick...", key)

        thread = threading.Thread(target=spam_loop, daemon=True)
        with self._lock:
            self._spam_threads[key] = (stop_event, thread)
        thread.start()

    def stop_spam(self, key: str) -> None:
        with self._lock:
            entry = self._spam_threads.pop(key, None)
        if entry is None:
            return
        stop_event, thread = entry
        stop_event.set()
        thread.join(timeout=1.0)
        logger.info("Spam generator thread terminated for key '%s'", key)

    def stop_all_spam(self) -> None:
        with self._lock:
            keys = list(self._spam_threads.keys())
        for key in keys:
            self.stop_spam(key)

    def is_spamming(self, key: str) -> bool:
        with self._lock:
            return key in self._spam_threads

    def set_subscriptions(self, cmds: set[int], filters: list[Filter]) -> None:
        with self._lock:
            self._filters = list(filters)
            removed = self._subscribed_cmds - cmds
            added = cmds - self._subscribed_cmds
            self._subscribed_cmds = set(cmds)

            for cmd in removed:
                topic = self._subscribers.pop(cmd, None)
                if topic is not None:
                    topic.unsubscribe()

            ros = self._ros
            if ros is not None and ros.is_connected:
                for cmd in added:
                    self._subscribe_locked(cmd, ros)

    def get_received_messages(self) -> list[ReceivedMessage]:
        with self._lock:
            return list(self._received)

    def clear_received_messages(self) -> None:
        with self._lock:
            self._received.clear()

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        self.stop_all_spam()
        with self._lock:
            for topic in self._subscribers.values():
                topic.unsubscribe()
            self._subscribers.clear()
            ros = self._ros
        if ros is not None:
            ros.terminate()
