import time
import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Joy
from kalman_wheels.crsf import (
    FRAME_LINK_STATISTICS, FRAME_RC_CHANNELS_PACKED,
    CrsfParser, normalize_channel, unpack_channels,
)


class CrsfJoy(Node):
    def __init__(self):
        super().__init__("crsf_joy")
        self.port = self.declare_parameter(
            "port", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00ZX3D-if00-port0"
        ).value
        self.baud = self.declare_parameter("baud", 420000).value
        self.rate = self.declare_parameter("rate", 50).value
        self.link_timeout = self.declare_parameter("link_timeout", 0.25).value

        self.joy_pub = self.create_publisher(Joy, "joy", 10)

        self.serial = None
        self.parser = CrsfParser()
        self.channels = None
        self.uplink_lq = None
        self.last_rc_time = 0.0
        self.last_open_attempt = 0.0
        self.link_up = False

        self.create_timer(0.005, self.poll)
        self.create_timer(1.0 / self.rate, self.publish)
    
    def open_serial(self):
        now = time.time()
        if now - self.last_open_attempt < 1.0:
            return
        self.last_open_attempt = now
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0)
            self.parser = CrsfParser()
            self.get_logger().info(f"Opened {self.port} @ {self.baud}")
        except serial.SerialException as e:
            self.serial = None
            self.get_logger().warn(f"Cannot open {self.port}: {e}", throttle_duration_sec=5.0)

    def poll(self):
        if self.serial is None:
            self.open_serial()
            return
        try:
            data = self.serial.read(self.serial.in_waiting or 1)
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Serial error: {e}")
            try:
                self.serial.close()
            except Exception:
                pass
            self.serial = None
            self.set_link(False)
            return

        now = time.time()
        for frame_type, payload in self.parser.feed(data):
            if frame_type == FRAME_RC_CHANNELS_PACKED and len(payload) >= 22:
                self.channels = [normalize_channel(v) for v in unpack_channels(payload)]
                self.last_rc_time = now
            elif frame_type == FRAME_LINK_STATISTICS and len(payload) >= 3:
                self.uplink_lq = payload[2]

        self.set_link(
            self.channels is not None
            and now - self.last_rc_time < self.link_timeout
            and self.uplink_lq != 0
        )

    def publish(self):
        if not self.link_up:
            return
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "elrs"
        msg.axes = self.channels
        self.joy_pub.publish(msg)

    def set_link(self, up: bool):
        if up != self.link_up:
            self.link_up = up
            if up:
                self.get_logger().info(f"ELRS link up (LQ {self.uplink_lq})")
            else:
                self.get_logger().warn("ELRS link lost")


def main():
    try:
        rclpy.init()
        rclpy.spin(CrsfJoy())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()