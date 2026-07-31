#!/usr/bin/env python3
# This runs on raspi as a service

import argparse
import fcntl
import os
import selectors
import signal
import socket
import sys
import termios


running = True


def stop(_signum, _frame):
    global running
    running = False


def baud_rate(value: int) -> int:
    rates = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
        230400: termios.B230400,
    }
    if value not in rates:
        raise ValueError(f"unsupported baud rate: {value}")
    return rates[value]


def open_serial(path: str, baud: int) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)

    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    attrs[3] = 0
    attrs[4] = baud_rate(baud)
    attrs[5] = baud_rate(baud)
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)

    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)
    return fd


def write_all(fd: int, data: bytes):
    view = memoryview(data)
    while view and running:
        try:
            written = os.write(fd, view)
            view = view[written:]
        except BlockingIOError:
            continue


def read_frames(from_fd: int, to_fd: int | None, frame: bytearray, eof_closes: bool):
    while True:
        try:
            data = os.read(from_fd, 512)
        except BlockingIOError:
            return

        if not data and eof_closes:
            raise EOFError("peer closed")
        if not data:
            return

        for byte in data:
            frame.append(byte)
            if byte == 0:
                if to_fd is not None:
                    write_all(to_fd, frame)
                frame.clear()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("serial_device")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    serial_fd = open_serial(args.serial_device, args.baud)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setblocking(False)
    server.bind(("", args.port))
    server.listen(1)

    selector = selectors.DefaultSelector()
    selector.register(server, selectors.EVENT_READ)
    selector.register(serial_fd, selectors.EVENT_READ)

    client = None
    serial_frame = bytearray()
    tcp_frame = bytearray()

    print(
        f"RSCP TCP serial server listening on port {args.port}, "
        f"serial {args.serial_device} @ {args.baud}"
    )

    try:
        while running:
            for key, _events in selector.select(timeout=0.1):
                if key.fileobj is server:
                    if client is not None:
                        selector.unregister(client)
                        client.close()
                    client, _address = server.accept()
                    client.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                    client.setsockopt(socket.IPPROTO_TCP, socket.SO_KEEPALIVE, 20)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, 5)
                    client.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, 3)

                    client.setblocking(False)
                    selector.register(client, selectors.EVENT_READ)
                    tcp_frame.clear()
                    print("Client connected")
                elif key.fileobj == serial_fd:
                    read_frames(
                        serial_fd,
                        client.fileno() if client is not None else None,
                        serial_frame,
                        False,
                    )
                elif client is not None and key.fileobj is client:
                    try:
                        read_frames(client.fileno(), serial_fd, tcp_frame, True)
                    except EOFError as error:
                        print(f"Client disconnected: {error}")
                        selector.unregister(client)
                        client.close()
                        client = None
                        tcp_frame.clear()
    finally:
        if client is not None:
            client.close()
        server.close()
        os.close(serial_fd)

    return 0


if __name__ == "__main__":
    sys.exit(main())
