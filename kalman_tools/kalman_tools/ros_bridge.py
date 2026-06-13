from __future__ import annotations

import atexit
import json
import os
import threading
import time
import urllib.request
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable

from roslibpy import Message, Ros, Topic

from kalman_tools.master_message_catalog import cmd_to_receive_topic

MASTER_MESSAGE_TYPE = "kalman_interfaces/msg/MasterMessage"
SEND_TOPIC = "master_com/ros_to_master"
MAX_LOG_SIZE = 500
DEBUG_LOG_PATH = "/home/wiktor/programming/students_research_group/agh_space_systems/kalman_ws/.cursor/debug-cebf79.log"
DEBUG_SESSION_ID = "cebf79"
DEBUG_INGEST_URL = "http://127.0.0.1:7506/ingest/07c2b1c4-cc73-4d5a-b1d6-172fb49d8897"


def _debug_log(hypothesis_id: str, location: str, message: str, data: dict) -> None:
    payload = {
        "sessionId": DEBUG_SESSION_ID,
        "runId": f"pid-{os.getpid()}",
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    try:
        request = urllib.request.Request(
            DEBUG_INGEST_URL,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Content-Type": "application/json",
                "X-Debug-Session-Id": DEBUG_SESSION_ID,
            },
            method="POST",
        )
        with urllib.request.urlopen(request, timeout=0.2):
            pass
    except Exception:
        pass
    try:
        with open(DEBUG_LOG_PATH, "a", encoding="utf-8") as debug_file:
            debug_file.write(json.dumps(payload, separators=(",", ":")) + "\n")
    except Exception:
        pass


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
        self._loop_iteration = 0

        # region agent log
        _debug_log(
            "H2",
            "ros_bridge.py:82",
            "MasterRosBridge init",
            {
                "instanceId": id(self),
                "host": self.host,
                "port": self.port,
                "thread": threading.current_thread().name,
            },
        )
        # endregion

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
        while not self._closed:
            ros: Ros | None = None
            try:
                self._loop_iteration += 1
                # region agent log
                _debug_log(
                    "H1",
                    "ros_bridge.py:107",
                    "_run_ros_loop iteration start",
                    {
                        "instanceId": id(self),
                        "loopIteration": self._loop_iteration,
                        "closed": self._closed,
                    },
                )
                # endregion
                ros = Ros(host=self.host, port=self.port)

                def on_ready(*_) -> None:
                    # region agent log
                    _debug_log(
                        "H4",
                        "ros_bridge.py:122",
                        "on_ready callback",
                        {
                            "instanceId": id(self),
                            "loopIteration": self._loop_iteration,
                            "argCount": len(_),
                        },
                    )
                    # endregion
                    if ros is None:
                        return
                    publisher = Topic(ros, SEND_TOPIC, MASTER_MESSAGE_TYPE)
                    publisher.advertise()
                    with self._lock:
                        self._ros = ros
                        self._publisher = publisher
                        self._resubscribe_locked()
                    self._set_connected(True)

                ros.on("ready", on_ready)
                # region agent log
                _debug_log(
                    "H1",
                    "ros_bridge.py:148",
                    "calling ros.run",
                    {
                        "instanceId": id(self),
                        "loopIteration": self._loop_iteration,
                    },
                )
                # endregion
                ros.run()
                # region agent log
                _debug_log(
                    "H5",
                    "ros_bridge.py:156",
                    "ros.run returned, stopping loop to avoid Twisted reactor restart",
                    {
                        "instanceId": id(self),
                        "loopIteration": self._loop_iteration,
                    },
                )
                # endregion
                break

            except Exception as exc:
                # region agent log
                _debug_log(
                    "H1",
                    "ros_bridge.py:160",
                    "_run_ros_loop exception",
                    {
                        "instanceId": id(self),
                        "loopIteration": self._loop_iteration,
                        "exceptionType": type(exc).__name__,
                        "exceptionMessage": str(exc),
                    },
                )
                # endregion
                self._set_connected(False)
                if exc.__class__.__name__ == "ReactorNotRestartable":
                    # region agent log
                    _debug_log(
                        "H5",
                        "ros_bridge.py:178",
                        "ReactorNotRestartable caught, breaking retry loop",
                        {
                            "instanceId": id(self),
                            "loopIteration": self._loop_iteration,
                        },
                    )
                    # endregion
                    break
            finally:
                with self._lock:
                    self._publisher = None
                    self._subscribers.clear()
                    self._ros = None
                self._set_connected(False)
                if ros is not None:
                    try:
                        # region agent log
                        _debug_log(
                            "H3",
                            "ros_bridge.py:180",
                            "calling ros.terminate",
                            {
                                "instanceId": id(self),
                                "loopIteration": self._loop_iteration,
                            },
                        )
                        # endregion
                        ros.terminate()
                    except Exception:
                        pass

            if not self._closed:
                time.sleep(1.0)

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
        publisher.publish(Message({"cmd": int(cmd), "data": [int(byte) for byte in data]}))

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
                    break

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
        # region agent log
        _debug_log(
            "H3",
            "ros_bridge.py:319",
            "close called",
            {
                "instanceId": id(self),
                "loopIteration": self._loop_iteration,
            },
        )
        # endregion
        self.stop_all_spam()
        with self._lock:
            for topic in self._subscribers.values():
                topic.unsubscribe()
            self._subscribers.clear()
            ros = self._ros
        if ros is not None:
            ros.terminate()
