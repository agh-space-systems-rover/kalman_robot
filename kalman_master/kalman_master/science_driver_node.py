import rclpy

from rclpy.node import Node

from kalman_interfaces.srv import Science
from kalman_interfaces.msg import MasterMessage

NUMBER_OF_RETRIES_PER_CALL = 10
PUBLISH_RATE = 30  # NOTE: only published when there are messages to send
MAX_MESSAGES_IN_QUEUE = 100  # About 3 seconds to process a full queue

# cmd data[]




class ScienceDriver(Node):
    def __init__(self):
        super().__init__("science_driver")
        self.master_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.create_service(Science, "request_science", self.request_science)
        self.msgs_to_send = []
        self.create_timer(1 / PUBLISH_RATE, self.tick)

    def request_science(self, req: Science.Request, res: Science.Response):
        # todo sanity check
        cmd = 0
        data = []
        msgs = []

        match req.cmd:
            case MasterMessage.SCIENCE_TARE_ROCKS:
                cmd = 0x01
                data = [0x23, 0x34]
            case MasterMessage.SCIENCE_REQUEST_ROCKS:
                cmd = 0x01
                data = [0x23, 0x35]
            case MasterMessage.SCIENCE_TARE_SAMPLE:
                cmd = 0x01
                data = [0x24, 0x34]
            case MasterMessage.SCIENCE_REQUEST_SAMPLE:
                cmd = 0x01
                data = [0x24, 0x35]
            case MasterMessage.SCIENCE_TARE_DRILL:
                cmd = 0x01
                data = [0x25, 0x34]
            case MasterMessage.SCIENCE_REQUEST_DRILL:
                cmd = 0x01
                data = [0x25, 0x35]
            case MasterMessage.SCIENCE_AUTONOMY_DRILL:
                cmd = 0x01
                data = [0x25, 0x36]

            case MasterMessage.CONTAINER1_CLOSE:
                cmd = 0x02
                data = [0x00, 0x00]
            case MasterMessage.CONTAINER1_OPEN:
                cmd = 0x02
                data = [0x00, 0x01]
            case MasterMessage.CONTAINER2_CLOSE:
                cmd = 0x02
                data = [0x01, 0x00]
            case MasterMessage.CONTAINER2_OPEN:
                cmd = 0x02
                data = [0x01, 0x01]
        
        msgs.append(
                MasterMessage(
                    cmd=cmd,
                    data=data,
                )
            )
        
        self.msgs_to_send.extend(msgs * NUMBER_OF_RETRIES_PER_CALL)
        self.msgs_to_send = self.msgs_to_send[-MAX_MESSAGES_IN_QUEUE:]

        res.weight=float(-1)
        return res

    def tick(self):
        if len(self.msgs_to_send) > 0:
            self.master_pub.publish(self.msgs_to_send.pop(0))




def main():
    try:
        rclpy.init()
        node = ScienceDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
