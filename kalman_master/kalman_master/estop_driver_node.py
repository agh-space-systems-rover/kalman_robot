import rclpy

from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from kalman_interfaces.msg import MasterMessage

NUMBER_OF_RETRIES_PER_CALL = 3
PUBLISH_RATE = 30  # NOTE: only published when there are messages to send
MAX_MESSAGES_IN_QUEUE = 100  # About 3 seconds to process a full queue


class EStopDriver(Node):
    def __init__(self):
        super().__init__("estop_driver")

        # Set
        self.set_state_srv = self.create_service(
            SetBool, "set_estop", self.set_estop_handler
        )
        self.msgs_to_send = []
        self.queue_send_timer = self.create_timer(
            1 / PUBLISH_RATE, self.send_msg_from_queue
        )
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )

        # Get
        self.master_sub = self.create_subscription(
            MasterMessage, "master_com/master_to_ros", self.master_frame_received, 10
        )
        self.last_state = None
        self.state_pub = self.create_publisher(Bool, "estop_state", 10)
        self.idle_state_pub_timer = self.create_timer(1.0, self.publish_state)

    def set_estop_handler(self, req: SetBool.Request, res: SetBool.Response):
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

        res.success = True
        return res

    def send_msg_from_queue(self):
        if len(self.msgs_to_send) > 0:
            self.master_pub.publish(self.msgs_to_send.pop(0))

    def master_frame_received(self, msg: MasterMessage):
        if msg.cmd == MasterMessage.MOTOR_GET_ESTOP:
            if msg.data[0] == 1:
                self.last_state = True
            elif msg.data[0] == 0:
                self.last_state = False
            else:
                self.get_logger().error("Invalid E-Stop state received.")
                return

            self.publish_state()

    def publish_state(self):
        if self.last_state is not None:
            self.state_pub.publish(Bool(data=self.last_state))


def main():
    try:
        rclpy.init()
        node = EStopDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
