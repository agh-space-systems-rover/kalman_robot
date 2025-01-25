#!/usr/bin/env python3
import struct
from serial import Serial, SerialException
from enum import Enum
from collections import deque
from dataclasses import dataclass

from rclpy.node import Node
from std_msgs.msg import String, Int16

UInt8 = int


@dataclass
class SerialMsg:
    cmd: UInt8
    argc: UInt8
    argv: list[UInt8]

    def as_list(self) -> list[UInt8]:
        return [self.cmd, self.argc, *self.argv]


class BinaryParserState(Enum):
    IDLE = 0
    CMD = 1
    LEN = 2
    HRC = 3
    DATA = 4
    CRC = 5
    URC = 6


class SerialDriver:
    def __init__(
        self,
        node: Node,
        port_name="/dev/ttyTHS2",
        start_byte="<",
        stop_byte=">",
        baud_rate=115200,
        tick_sleep_time_s=0.01,
        ascii_mode=False,
    ) -> None:
        """
        Initializes a new instance of the SerialDriver class.

        Args:
            port_name (str, optional): The name of the serial port to use. Defaults to '/dev/ttyTHS2'.
            _START_BYTE (str, optional): The start byte used to delimit messages. Defaults to '<'.
            _STOP_BYTE (str, optional): The stop byte used to delimit messages. Defaults to '>'.
            _BAUD (int, optional): The baud rate to use. Defaults to 115200.
            tick_sleep_time_s (float, optional): The amount of time to sleep between ticks. Defaults to 0.01.
            ascii_mode (bool, optional): Whether to use ASCII mode. Defaults to False.
        """
        self.node = node
        self.serial_read_buffer: bytes = b""
        self.serial_write_buffer: deque[bytes] = deque()
        self.ros_msg_read_buffer: list[SerialMsg] = []
        self._START_BYTE: str = start_byte
        self._STOP_BYTE: str = stop_byte
        self._START_BYTE_ENCODED = self._START_BYTE.encode()
        self._STOP_BYTE_ENCODED = self._STOP_BYTE.encode()

        self.serial = Serial(port=port_name, baudrate=baud_rate, timeout=0.01)

        self.malformed_packets_timer_time = 30
        self.timer_log_data = node.create_timer(1, self.log_data_speed)
        self.timer_log_malformed_packets = node.create_timer(
            self.malformed_packets_timer_time, self.log_malformed_packets
        )

        self.baudrate_debug_pub = node.create_publisher(
            String, "master_com/baudrate_debug", 10
        )
        self.malformed_packets_pub = node.create_publisher(
            Int16,
            f"master_com/malformed_packets_last_{self.malformed_packets_timer_time}_secs",
            10,
        )
        self.bitrate_tx = 0
        self.bitrate_rx = 0
        self.correct_bitrate_rx = 0
        self.malformed_packets = 0

        self.ascii_mode = ascii_mode

        self.binary_parser_state: BinaryParserState = BinaryParserState.IDLE
        self.binary_parser_to_read = 0
        self.binary_parser_msg_buffer: list[UInt8] = []
        self.binary_parser_crc_correct = False

        self.on_error_sleep_rate = None

    def get_logger(self):
        return self.node.get_logger()

    def destroy(self) -> None:
        """
        Finalizes all destryable resources used by the driver.
        The object should not be used after this step.
        Please create another instance of SerialDriver instead.

        Returns:
            None
        """
        if self.on_error_sleep_rate is not None:
            self.on_error_sleep_rate.destroy()
            self.node.destroy_rate(self.on_error_sleep_rate)
        self.node.destroy_publisher(self.malformed_packets_pub)
        self.node.destroy_publisher(self.baudrate_debug_pub)
        self.node.destroy_timer(self.timer_log_malformed_packets)
        self.node.destroy_timer(self.timer_log_data)
        self.serial.close()

    def log_malformed_packets(self) -> None:
        """
        Logs the number of malformed packets received in the last malformed_packets_timer_time seconds.
        Used as callback for a timer.

        Returns:
            None
        """
        msg = String(
            data=f"malformed packets in last {self.malformed_packets_timer_time} seconds: {self.malformed_packets}"
        )
        self.baudrate_debug_pub.publish(msg)
        self.malformed_packets_pub.publish(Int16(data=self.malformed_packets))
        self.malformed_packets = 0

    def log_data_speed(self) -> None:
        """
        Logs the number of bytes read and written per second.
        Used as callback for a timer.

        Returns:
            None
        """
        self.baudrate_debug_pub.publish(String(data=f"data TX {self.bitrate_tx}"))
        self.baudrate_debug_pub.publish(String(data=f"data RX {self.bitrate_rx}"))
        self.baudrate_debug_pub.publish(
            String(data=f"correct data RX {self.correct_bitrate_rx}")
        )
        self.bitrate_tx = 0
        self.bitrate_rx = 0
        self.correct_bitrate_rx = 0

    def write_msg(self, msg: SerialMsg) -> None:
        """
        Write a message to the serial_write_buffer.

        Args:
            msg (SerialMsg): The message to write.

        Returns:
            None
        """
        if self.ascii_mode:
            encoded_msg = self._encode_msg(msg)
        else:
            encoded_msg = self._encode_msg_binary(msg)
        self.serial_write_buffer.append(encoded_msg)

    def tick(self):
        """
        Read and write bytes from/to the serial device.
        """
        self._write_to_serial()
        if self.ascii_mode:
            self._read_from_serial()
        else:
            self._read_from_serial_binary()

    def read_all_msgs(self) -> list[SerialMsg]:
        """
        Read all messages from the ros_msg_read_buffer.

        Returns:
            list[SerialMsg]: The messages read from the ros_msg_read_buffer.
        """
        msgs = self.ros_msg_read_buffer[:]
        self.ros_msg_read_buffer = []
        return msgs

    def _init_port(self) -> None:
        """
        Initialize the serial port.
        """
        self.on_error_sleep_rate = self.node.create_rate(1)
        while True:
            try:
                self.serial.open()
                break
            except SerialException as e:
                self.get_logger().error(e)
                self.on_error_sleep_rate.sleep()

    def _read_from_serial(self) -> None:
        """
        Read bytes from serial using _validate_decode_and_store_packet to parse them.

        Gets all bytes from the serial device and stores them in the serial_read_buffer.
        For each start byte found, it tries to parse a packet.
        """
        if not self.serial.in_waiting:
            return

        new_bytes = self.serial.read_all()
        self.bitrate_rx += 8 * len(new_bytes)
        self.serial_read_buffer += new_bytes
        start_byte_idx = self.serial_read_buffer.find(self._START_BYTE_ENCODED)
        iterations = 0
        # iterations -- sanity check in case of unexpected blocking
        while not start_byte_idx == -1 and iterations < 20:
            iterations += 1
            self.serial_read_buffer = self.serial_read_buffer[start_byte_idx:]
            stop_byte_idx = self.serial_read_buffer.find(self._STOP_BYTE_ENCODED)
            if stop_byte_idx != -1:
                packet = self.serial_read_buffer[: stop_byte_idx + 1]
                start_byte_idx = packet[1:].find(self._START_BYTE_ENCODED)

                # found start byte in the middle of the packet -> malformed packet
                if start_byte_idx != -1 and start_byte_idx < stop_byte_idx:
                    self.serial_read_buffer = self.serial_read_buffer[start_byte_idx:]
                    iterations += 1
                    self.malformed_packets += 1
                    self.get_logger().error(
                        "malformed packet, double start byte without end byte"
                    )
                    continue
                self._validate_decode_and_store_packet(packet)
                self.serial_read_buffer = self.serial_read_buffer[stop_byte_idx + 1 :]
                start_byte_idx = self.serial_read_buffer.find(self._START_BYTE_ENCODED)
            else:
                break
        if iterations == 20:
            self.serial_read_buffer = b""
            self.get_logger().error("iterations == 20, that is not a good sign")

    def _validate_decode_and_store_packet(self, packet: bytes) -> None:
        """
        Validate, decode and store a packet given in ASCII format.

        Decodes from ASCII hex to integers.
        Saves the decoded packet in the ros_msg_read_buffer.

        Args:
            packet (bytes): The packet to validate, decode and store.

        Returns:
            None
        """
        if len(packet) < 2:
            self.get_logger().error("received packet smaller than 2 ascii signs")
            self.malformed_packets += 1
            return

        packet = packet[1:-1]
        result: list[UInt8] = []
        for i in range(0, len(packet), 2):
            try:
                byte = int(packet[i : i + 2], 16)
            except Exception as e:
                self.get_logger().error(e)
                byte = 0
            result.append(byte)

        if len(result) < 2:
            self.get_logger().error("received packet smaller than 2 bytes")
            self.malformed_packets += 1
            return

        if self._crc_valid(result):
            # *8*2 bo zamieniamy bity na bajty, a kazdy bajt jest kodowany jako 2 znaki ascii
            # dodajemy 2*8 bo to '<' i '>'
            self.correct_bitrate_rx += len(result) * 8 * 2 + 2 * 8
            msg = SerialMsg(cmd=result[0], argc=result[1], argv=result[2:-1])
            self.ros_msg_read_buffer.append(msg)
        else:
            self.malformed_packets += 1

    def _read_from_serial_binary(self) -> None:
        """
        Read bytes from the serial device using _parse_packet_binary to parse them.

        Returns:
            None
        """
        if not self.serial.in_waiting:
            return

        new_bytes = self.serial.read_all()
        self.bitrate_rx += 8 * len(new_bytes)

        # Process packet byte by byte
        for byte in new_bytes:
            self._parse_packet_binary(byte)

    def _parse_packet_binary(self, byte: UInt8) -> None:
        """
        Parse a packet byte by byte. Using a state machine to keep track of the current state.

        State machine states:
        IDLE: Waiting for start byte
        CMD: Waiting for command byte
        LEN: Waiting for length byte
        HRC: Waiting for header control byte
        DATA: Waiting for data bytes
        CRC: Waiting for CRC byte
        URC: Waiting for user return control byte

        In case of a malformed packet, the state machine will return to the IDLE state.
        Final messages are stored in the ros_msg_read_buffer.

        Args:
            byte (UInt8): The byte to parse.

        Returns:
            None
        """

        if self.binary_parser_state == BinaryParserState.IDLE:
            if byte == self._START_BYTE_ENCODED[0]:
                self.binary_parser_state = BinaryParserState.CMD

                # Reset parser to process new message
                self.binary_parser_to_read = 0
                self.binary_parser_msg_buffer.clear()

        elif self.binary_parser_state == BinaryParserState.CMD:
            self.binary_parser_msg_buffer.append(byte)

            self.binary_parser_state = BinaryParserState.LEN

        elif self.binary_parser_state == BinaryParserState.LEN:
            self.binary_parser_msg_buffer.append(byte)
            self.binary_parser_to_read = byte

            self.binary_parser_state = BinaryParserState.HRC

            # check if packet is not too long, only if not Custom packet
            if byte > 16 and (
                self.binary_parser_msg_buffer[-2] < 128
                or self.binary_parser_msg_buffer[-2] > 131
            ):
                self.get_logger().error(
                    f"Packet too long, not dropping {self.binary_parser_msg_buffer}"
                )
                self.malformed_packets += 1
                # self.binary_parser_state = BinaryParserState.IDLE

        elif self.binary_parser_state == BinaryParserState.HRC:
            if byte == self._calc_hrc(self.binary_parser_msg_buffer):
                if self.binary_parser_msg_buffer[-1] > 0:
                    self.binary_parser_state = BinaryParserState.DATA
                else:
                    self.binary_parser_state = BinaryParserState.CRC
            else:
                self.get_logger().error(
                    f"HRC not valid for packet {self.binary_parser_msg_buffer}"
                )
                self.get_logger().error(
                    f"HRC byte is {byte} , should be {self._calc_hrc(self.binary_parser_msg_buffer)}"
                )

                self.malformed_packets += 1
                self.binary_parser_state = BinaryParserState.IDLE

        elif self.binary_parser_state == BinaryParserState.DATA:
            self.binary_parser_msg_buffer.append(byte)

            self.binary_parser_to_read -= 1
            if self.binary_parser_to_read == 0:
                self.binary_parser_state = BinaryParserState.CRC

        elif self.binary_parser_state == BinaryParserState.CRC:
            if byte == self._calc_crc(self.binary_parser_msg_buffer):
                self.binary_parser_crc_correct = True
                self.binary_parser_state = BinaryParserState.URC
            else:
                self.get_logger().error(
                    f"CRC not valid for packet {self.binary_parser_msg_buffer}"
                )
                self.get_logger().error(
                    f"CRC byte is {byte} , should be {self._calc_crc(self.binary_parser_msg_buffer)}"
                )

                self.malformed_packets += 1

                self.binary_parser_crc_correct = False
                self.binary_parser_state = BinaryParserState.URC
        elif self.binary_parser_state == BinaryParserState.URC:
            if not self.binary_parser_crc_correct:
                pass
            elif byte == self._calc_urc(self.binary_parser_msg_buffer):
                self.correct_bitrate_rx += len(self.binary_parser_msg_buffer) * 8 + 8

                msg = SerialMsg(
                    cmd=self.binary_parser_msg_buffer[0],
                    argc=self.binary_parser_msg_buffer[1],
                    argv=self.binary_parser_msg_buffer[2:],
                )
                self.ros_msg_read_buffer.append(msg)

            else:
                self.get_logger().error(
                    f"URC not valid for packet {self.binary_parser_msg_buffer}"
                )
                self.get_logger().error(
                    f"URC byte is {byte} , should be {self._calc_urc(self.binary_parser_msg_buffer)}"
                )

                self.malformed_packets += 1

            self.binary_parser_state = BinaryParserState.IDLE

    def _write_to_serial(self) -> None:
        """
        Write bytes from the serial_write_buffer to the serial device.

        Returns:
            None
        """
        if len(self.serial_write_buffer) > 0:
            bytes_to_send = self.serial_write_buffer.popleft()
            self.bitrate_tx += len(bytes_to_send) * 8
            self.serial.write(bytes_to_send)

    def _encode_msg(self, msg: SerialMsg) -> bytes:
        """
        Encode a ROS message as a packet. The packet is encoded as follows:
        [START_BYTE, CMD, LEN, ARG_0, ..., ARG_N, CRC, STOP_BYTE]

        Args:
            msg (SerialMsg): The ROS message to encode.

        Returns:
            bytes: The encoded message.
        """
        data: list[UInt8] = msg.as_list()
        crc = self._calc_crc(data)
        data.append(crc)
        payload = "".join([self._byte2hex(byte) for byte in data])
        payload = self._START_BYTE + payload + self._STOP_BYTE
        return payload.encode("ascii")

    def _byte2hex(self, _byte: UInt8) -> bytes:
        """
        Convert a byte to a hex string, without the '0x' prefix, and padded to 2 characters,
        so there are leading zeros. (e.g. 0x0A -> '0A')

        Args:
            _byte (UInt8): The byte to convert.

        Returns:
            bytes: The hex string.
        """
        return hex(_byte)[2:].rjust(2, "0")

    def _encode_msg_binary(self, msg: SerialMsg) -> bytes:
        """
        Encode a ROS message as a binary packet. The packet is encoded as follows:
        [START_BYTE, CMD, LEN, HRC, ARG_0, ..., ARG_N, CRC, URC]

        Args:
            msg (Serial): The ROS message to encode.

        Returns:
            bytes: The encoded message.
        """
        data: list[UInt8] = msg.as_list()
        hrc = self._calc_hrc(data)
        crc = self._calc_crc(data)
        urc = self._calc_urc(data)
        data = [*data[0:2], hrc, *data[2:]]
        data.append(crc)
        data.append(urc)
        payload = [ord(self._START_BYTE), *data]
        return struct.pack("B" * len(payload), *payload)

    def _calc_hrc(self, data: list[UInt8]) -> UInt8:
        """
        Calculate control and return control sum for a new packet.

        Args:
            data (list[UInt8]): The data to calculate the HRC for.

        Returns:
            UInt8: The HRC.
        """
        hrc = 0x00
        for arg in data[0:2]:
            hrc += arg
            hrc %= 256
        return hrc % 256

    def _calc_crc(self, data: list[UInt8]) -> UInt8:
        """
        Calculate control and return control sum for a new packet.

        Args:
            data (list[UInt8]): The data to calculate the CRC for.

        Returns:
            UInt8: The CRC.
        """
        crc = 0x00
        for arg in data:
            crc ^= arg
        return crc

    def _calc_urc(self, data: list[UInt8]) -> UInt8:
        """
        Calculate control and return control sum for a new packet.

        Args:
            data (list[UInt8]): The data to calculate the URC for.

        Returns:
            UInt8: The URC.
        """
        crc = 0x00
        for i, arg in enumerate(data[2:]):
            crc += (arg + i) * ((i % 4) + 1)
            crc %= 256
        return crc % 256

    def _crc_valid(self, data: list[UInt8]) -> bool:
        """
        Check if the CRC of the data is valid.

        Args:
            data (list[UInt8]): The data to check the CRC of (last element is CRC).

        Returns:
            bool: Whether the CRC is valid.
        """
        crc = self._calc_crc(data[:-1])
        return crc == data[-1]
