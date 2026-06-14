from __future__ import annotations

import atexit
import logging
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable

from roslibpy import Message, Ros, Topic

from kalman_tools.master_message_catalog import cmd_to_receive_topic

# Set up logging for background thread diagnostics
logger = logging.getLogger(__name__)

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
            self._on_connection_change(connected)

    def _run_ros_loop(self) -> None:
        ros: Ros | None = None
        try:
            # max_reconnection_attempts=-1: roslibpy retries inside the same
            # Twisted reactor run — no reactor restart needed.
            ros = Ros(
                host=self.host,
                port=self.port,
                retry_startup_delay=2.0,
                max_reconnection_attempts=-1,
            )

            def on_ready(*_) -> None:
                publisher = Topic(ros, SEND_TOPIC, MASTER_MESSAGE_TYPE)
                publisher.advertise()
                with self._lock:
                    self._ros = ros
                    self._publisher = publisher
                    # clear any stale subscribers from a previous connection
                    self._subscribers.clear()
                    self._resubscribe_locked()
                self._set_connected(True)

            def on_close(*_) -> None:
                with self._lock:
                    self._publisher = None
                    self._subscribers.clear()
                    self._ros = None
                self._set_connected(False)

            ros.on("ready", on_ready)
            ros.on("close", on_close)
            ros.run()  # blocks until ros.terminate() is called

        except Exception as e:
            logger.exception("Exception occurred in the core ROS loop thread: %s", e)
        finally:
            with self._lock:
                self._publisher = None
                self._subscribers.clear()
                self._ros = None
            self._set_connected(False)
            if ros is not None:
                try:
                    ros.terminate()
                except Exception:
                    pass

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
            lambda message, cmd=cmd, topic_name=topic_name: self._on_message(
                cmd, topic_name, message
            )
        )
        self._subscribers[cmd] = topic

    def _on_message(self, cmd: int, topic: str, message: dict) -> None:
        data = [int(byte) for byte in message.get("data", [])]
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
            if filt.op == "==":
                if left != right:
                    return False
            elif filt.op == "!=":
                if left == right:
                    return False
            elif filt.op == "<":
                if not left < right:
                    return False
            elif filt.op == ">":
                if not left > right:
                    return False
            elif filt.op == "<=":
                if not left <= right:
                    return False
            elif filt.op == ">=":
                if not left >= right:
                    return False
            else:
                return False
        return True

    def send_frame(self, cmd: int, data: list[int]) -> None:
        with self._lock:
            publisher = self._publisher
        if publisher is None:
            raise RuntimeError("Not connected to rosbridge")
        publisher.publish(Message({"cmd": int(cmd), "data": [int(b) for b in data]}))

    def start_spam(self, key: str, cmd: int, data: list[int], interval_s: float) -> None:
        if interval_s <= 0:
            raise ValueError("Spam interval must be positive")
        self.stop_spam(key)

        stop_event = threading.Event()

        def spam_loop() -> None:
            while not stop_event.wait(interval_s):
                try:
                    self.send_frame(cmd, data)
                except RuntimeError:
                    # FIX: Do not drop the thread completely. Network drops are expected,
                    # so log the incident and gracefully wait for the next iteration window.
                    logger.warning("Spam thread '%s' failed to write frame (disconnected). Retrying next frame tick...", key)

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
