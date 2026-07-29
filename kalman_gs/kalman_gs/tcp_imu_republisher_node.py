import math

import rclpy
import zmq
from rclpy.node import Node
from std_msgs.msg import Float64


SERVER_IP = "192.168.1.149"
ZMQ_PORT = 5001
REQUEST_INTERVAL_S = 3.0
RECEIVE_TIMEOUT_S = 0.5
IMU_NORTH_TOPIC = "/imu/north"


class TcpImuRepublisher(Node):
    def __init__(self) -> None:
        super().__init__("tcp_imu_republisher")
        self.north_pub = self.create_publisher(Float64, IMU_NORTH_TOPIC, 10)

        self.zmq_context = zmq.Context()
        self.zmq_socket = self.zmq_context.socket(zmq.SUB)
        self.zmq_socket.setsockopt(zmq.CONFLATE, 1)
        self.zmq_socket.setsockopt_string(zmq.SUBSCRIBE, "")
        self.zmq_socket.setsockopt(zmq.RCVTIMEO, int(RECEIVE_TIMEOUT_S * 1000))
        self.zmq_socket.connect(f"tcp://{SERVER_IP}:{ZMQ_PORT}")

        self.get_logger().info(
            f"Listening for ZMQ stream at tcp://{SERVER_IP}:{ZMQ_PORT}"
        )
        self.create_timer(REQUEST_INTERVAL_S, self.request_angle)

    def request_angle(self) -> None:
        try:
            payload = self.zmq_socket.recv_json()
        except (ValueError, zmq.ZMQError, zmq.Again):
            return

        if not isinstance(payload, dict):
            return

        try:
            angle = float(payload["angle"])
        except (KeyError, TypeError, ValueError, OverflowError):
            return

        self.north_pub.publish(Float64(data=angle))

    def destroy_node(self) -> None:
        self.zmq_socket.close(linger=0)
        self.zmq_context.term()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TcpImuRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
