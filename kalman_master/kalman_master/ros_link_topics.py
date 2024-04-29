from __future__ import annotations

import traceback
import time

from kalman_master.ros_link_serialization import serialize_message, deserialize_message

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from kalman_master.ros_link_node import RosLink


class RosLinkTopics:
    def __init__(self, node: RosLink):
        self.node = node

        # Subscribe to messages published on this side.
        for topic_config in self.node.get_this_side_config()["topics"]:
            self.node.create_subscription(
                self.node.imported_interfaces[topic_config["type"]],
                topic_config["name"],
                lambda msg, topic_config=topic_config: self.subscriber_callback(
                    msg, topic_config
                ),
                10,
                callback_group=self.node.callback_group,
            )

        # Create publishers for messages received from the other side.
        # Topics specified in the PC config are published by the PC and vice versa.
        # That's why we invert the PC and GS config here.
        self.topic_publishers = {}
        for topic_config in self.node.get_opposite_side_config()["topics"]:
            self.topic_publishers[topic_config["name"]] = self.node.create_publisher(
                self.node.imported_interfaces[topic_config["type"]],
                topic_config["name"]
                + ("/loopback" if self.node.loopback_mangling else ""),
                10,
            )

        self.last_msg_times: dict[str, float] = {}

    # Get the configuration for a topic.
    def get_topic_config(self, name: str) -> dict:
        for side_config in [
            self.node.get_this_side_config(),
            self.node.get_opposite_side_config(),
        ]:
            for topic_config in side_config["topics"]:
                if topic_config["name"] == name:
                    return topic_config
        raise ValueError(f"No configuration found for topic {name}.")

    # Called by master_callback when a topic frame is received.
    def handle_topic_frame(self, topic_name: str, data: list) -> None:
        # Deserialize the message.
        topic_config = self.get_topic_config(topic_name)
        try:
            topic_msg = deserialize_message(
                data,
                self.node.imported_interfaces[topic_config["type"]],
                topic_config["fields"],
            )
        except ValueError as e:
            self.node.get_logger().error(
                f"Failed to deserialize message on topic {topic_config['name']}:\n{e}"
            )
            return
        except Exception:
            self.node.get_logger().error(
                f"Unexpected error while deserializing a message on topic {topic_config['name']}:\n{traceback.format_exc()}"
            )
            return

        # Publish the message on the topic.
        self.topic_publishers[topic_name].publish(topic_msg)

    # Called whenever a message is received on a shared topic.
    # The message is serialized and sent to the master.
    def subscriber_callback(self, msg, topic_config: dict) -> None:
        # Rate limit the messages.
        if topic_config["name"] in self.last_msg_times:
            if (
                time.time() - self.last_msg_times[topic_config["name"]]
                < 1 / topic_config["max_rate"]
            ):
                return
        self.last_msg_times[topic_config["name"]] = time.time()

        # Determine the ID of the topic.
        topic_id = self.node.ordered_names.index(topic_config["name"])

        # Serialize the message.
        try:
            data = serialize_message(msg, topic_config["fields"])
        except ValueError as e:
            self.node.get_logger().error(
                f"Failed to serialize message on topic {topic_config['name']}:\n{e}"
            )
            return
        except:
            self.node.get_logger().error(
                f"Unexpected error while serializing a message on topic {topic_config['name']}:\n{traceback.format_exc()}"
            )
            return

        # Publish the serialized message to the master.
        self.node.send_to_opposite_side([topic_id] + data)
