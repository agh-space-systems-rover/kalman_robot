import rclpy

from rclpy.node import Node

from kalman_interfaces.srv import SetFeed
from kalman_interfaces.msg import MasterMessage

NUMBER_OF_RETRIES_PER_CALL = 1
PUBLISH_RATE = 30  # NOTE: only published when there are messages to send
MAX_MESSAGES_IN_QUEUE = 100  # About 3 seconds to process a full queue


class FeedDriver(Node):
    def __init__(self):
        super().__init__("feed_driver")

        # Init master publisher.
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        # Init services.
        self.create_service(SetFeed, "set_feed", self.set_feed)

        self.msgs_to_send = []
        self.create_timer(1 / PUBLISH_RATE, self.tick)

    def set_feed(self, req: SetFeed.Request, res: SetFeed.Response):
        if req.feed < 1 or req.feed > 2:
            return res
        if req.camera < 0 or req.camera > 8:
            # Cameras are indexed 1 to 8, zero means no change.
            return res
        if req.channel < 0 or req.channel > 40:
            return res
        if req.power < 0 or req.power > 4:
            return res

        msgs = []

        if req.camera != 0:
            self.get_logger().info(
                f"Setting camera {req.camera} for feed {req.feed}. ({NUMBER_OF_RETRIES_PER_CALL} times)"
            )
            msgs.append(
                MasterMessage(
                    cmd=MasterMessage.FEED_SET_CAMERA,
                    data=[req.feed, req.camera],  # Firmware indexes cameras from 1.
                )
            )
        if req.channel != 0:
            self.get_logger().info(
                f"Setting channel {req.channel} for feed {req.feed}. ({NUMBER_OF_RETRIES_PER_CALL} times)"
            )
            msgs.append(
                MasterMessage(
                    cmd=MasterMessage.FEED_SET_CHANNEL,
                    data=[req.feed, req.channel],  # Firmware indexes channels from 1.
                )
            )
        if req.power != 0:
            self.get_logger().info(
                f"Setting power {req.power} for feed {req.feed}. ({NUMBER_OF_RETRIES_PER_CALL} times)"
            )
            msgs.append(
                MasterMessage(
                    cmd=MasterMessage.FEED_SET_POWER,
                    data=[req.feed, req.power],  # Firmware indexes power from 1.
                )
            )

        # Multiply the messages by the number of retries.
        self.msgs_to_send.extend(msgs * NUMBER_OF_RETRIES_PER_CALL)
        # Limit the number of messages in the queue.
        # Drop the oldest messages
        self.msgs_to_send = self.msgs_to_send[-MAX_MESSAGES_IN_QUEUE:]

        return res

    def tick(self):
        if len(self.msgs_to_send) > 0:
            self.master_pub.publish(self.msgs_to_send.pop(0))


def main():
    try:
        rclpy.init()
        node = FeedDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
