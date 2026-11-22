"""Test of launch/tunnel.launch.py in each mode (gs, pc, arm).

master_com is connected to a virtual serial port (pty), standing in for the
Master. Checks that each mode starts exactly master_com and the right ros_link
sides with the right parameters, and that MasterMessages cross the serial port
in both directions.

The expected frame bytes were produced by serial_driver.py before it was moved
from kalman_master, so a change to the serial protocol is caught here too.
"""

import os
import pty
import time
import tty
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python import get_package_share_path
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

from kalman_interfaces.msg import MasterMessage

TIMEOUT = 20.0

BAUD_RATES = {"gs": 38400, "pc": 115200, "arm": 2000000}
# Expected ros_link nodes per mode: name -> (side, rover_endpoint).
ROS_LINKS = {
    "gs": {
        "ros_link_gs_to_pc": ("station", "pc"),
        "ros_link_gs_to_arm": ("station", "arm"),
    },
    "pc": {"ros_link_pc_to_gs": ("rover", "pc")},
    "arm": {"ros_link_arm_to_gs": ("rover", "arm")},
}

# UEUOS_SET_STATE (0x70) with data [2]: start, cmd, len, HRC, data, CRC, URC.
OUT_FRAME = MasterMessage(cmd=0x70, data=[2])
OUT_BYTES = bytes.fromhex("3c 70 01 71 02 73 02")
# LASER_DIST (0x5A) with data [1, 0, 0xd2, 0x04].
IN_BYTES = bytes.fromhex("3c 5a 04 5e 01 00 d2 04 89 9b")
IN_FRAME = MasterMessage(cmd=0x5A, data=[1, 0, 0xD2, 0x04])


@pytest.mark.launch_test
@launch_testing.parametrize("mode", ["gs", "pc", "arm"])
def generate_test_description(mode):
    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    os.set_blocking(master_fd, False)
    port = os.ttyname(slave_fd)

    tunnel = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("kalman_tunnel") / "launch/tunnel.launch.py")
        ),
        launch_arguments={"mode": mode, "port": port}.items(),
    )
    return (
        launch.LaunchDescription([tunnel, launch_testing.actions.ReadyToTest()]),
        {"master_fd": master_fd, "slave_fd": slave_fd, "port": port},
    )


class TestTunnelLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("tunnel_launch_test")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=TIMEOUT, every=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if every is not None:
                every()
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def launched_nodes(self):
        return {
            name
            for name in self.node.get_node_names()
            if name != self.node.get_name() and not name.startswith("launch_ros_")
        }

    def get_params(self, node_name, names):
        client = self.node.create_client(GetParameters, f"/{node_name}/get_parameters")
        try:
            self.assertTrue(
                client.wait_for_service(timeout_sec=TIMEOUT),
                f"{node_name} has no parameter service",
            )
            future = client.call_async(GetParameters.Request(names=names))
            self.assertTrue(self.spin_until(future.done), f"{node_name} did not answer")
            values = future.result().values
            # Only integer and string parameters are checked here.
            return {
                name: value.integer_value
                if value.type == ParameterType.PARAMETER_INTEGER
                else value.string_value
                for name, value in zip(names, values)
            }
        finally:
            self.node.destroy_client(client)

    def test_starts_exactly_the_tunnel_nodes(self, mode):
        expected = {"master_com", *ROS_LINKS[mode]}
        self.spin_until(lambda: expected <= self.launched_nodes())
        self.assertEqual(self.launched_nodes(), expected)

    def test_master_com_params(self, mode, port):
        params = self.get_params("master_com", ["baud_rate", "port"])
        self.assertEqual(params["baud_rate"], BAUD_RATES[mode])
        self.assertEqual(params["port"], port)

    def test_ros_link_params(self, mode):
        for name, (side, endpoint) in ROS_LINKS[mode].items():
            params = self.get_params(name, ["side", "rover_endpoint", "config_path"])
            self.assertEqual(params["side"], side, name)
            self.assertEqual(params["rover_endpoint"], endpoint, name)
            self.assertTrue(
                params["config_path"].endswith(f"kalman_tunnel/config/{endpoint}_ros_link.yaml"),
                f"{name}: {params['config_path']}",
            )

    def test_ros_to_serial(self, master_fd):
        pub = self.node.create_publisher(MasterMessage, "master_com/ros_to_master", 10)
        wire = bytearray()

        def publish_and_read():
            pub.publish(OUT_FRAME)
            try:
                wire.extend(os.read(master_fd, 4096))
            except BlockingIOError:
                pass

        try:
            ok = self.spin_until(lambda: OUT_BYTES in wire, every=publish_and_read)
            self.assertTrue(ok, f"expected {OUT_BYTES.hex(' ')}, got {bytes(wire[:64]).hex(' ')}")
        finally:
            self.node.destroy_publisher(pub)

    def test_serial_to_ros(self, master_fd):
        received = []
        sub = self.node.create_subscription(
            MasterMessage, "master_com/master_to_ros/x5a", received.append, 10
        )
        try:
            ok = self.spin_until(lambda: received, every=lambda: os.write(master_fd, IN_BYTES))
            self.assertTrue(ok, "no frame arrived on master_com/master_to_ros/x5a")
            self.assertEqual(received[0].cmd, IN_FRAME.cmd)
            self.assertEqual(list(received[0].data), list(IN_FRAME.data))
        finally:
            self.node.destroy_subscription(sub)


@launch_testing.post_shutdown_test()
class TestTunnelShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # SIGINT on shutdown; every node must exit cleanly.
        launch_testing.asserts.assertExitCodes(proc_info)
