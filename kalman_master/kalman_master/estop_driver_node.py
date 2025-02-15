import rclpy

from rclpy.node import Node

from std_srvs.srv import SetBool
from kalman_interfaces.msg import MasterMessage

NUMBER_OF_RETRIES_PER_CALL = 3
PUBLISH_RATE = 30  # NOTE: only published when there are messages to send
MAX_MESSAGES_IN_QUEUE = 100  # About 3 seconds to process a full queue


class EStopDriver(Node):
    def __init__(self):
        super().__init__("estop_driver")

        # Init master publisher.
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        # Init services.
        self.create_service(SetBool, "set_estop", self.set_estop)

        self.msgs_to_send = []
        self.create_timer(1 / PUBLISH_RATE, self.tick)

    def set_estop(self, req: SetBool.Request, res: SetBool.Response):
        self.get_logger().info(
            f"Setting E-Stop {'ON' if req.data else 'OFF'}. ({NUMBER_OF_RETRIES_PER_CALL} times)"
        )
        msgs = [
            MasterMessage(
                cmd=MasterMessage.MOTOR_SET_ESTOP,
                data=[1 if req.data else 0],
            )
        ]

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
        node = EStopDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
