"""Test of launch/drivers.launch.py in each mode (gs, pc, arm).

No master_com runs here: the test node stands in for it on the master_com
topics. Checks that each mode starts exactly its keep-list of drivers (no
dropped science/drill/camera_2dof drivers and no transport), that the
tunnel node serves /tunnel/* and sends its frames, and that the wheel and arm
twist drivers clamp out-of-range commands instead of crashing or wrapping.
Also pins the master_com topic names the drivers use, written out by hand.
"""

import math
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from ament_index_python import get_package_share_path
from launch.actions import IncludeLaunchDescription, TimerAction
from geometry_msgs.msg import Twist
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from std_srvs.srv import Empty

from kalman_interfaces.msg import MasterMessage, WheelState, WheelStates

TIMEOUT = 20.0
# How long to keep watching for extra nodes once the expected ones are up.
SETTLE = 2.0

# Node names started in each mode.
KEEP_NODES = {
    "gs": {
        "wheel_driver",
        "ueuos_driver",
        "feed_driver",
        "arm_twist_driver",
        "rfid_driver",
        "tunnel",
        "laser_driver_node",
    },
    "pc": {
        "wheel_driver",
        "ueuos_driver",
        "autonomy_switch_spam",
        "laser_driver_node",
    },
    "arm": set(),
}

# master_com topics each driver must use: node -> [(topic, "pub" or "sub")].
MASTER_TOPICS = {
    "wheel_driver": [
        ("/master_com/ros_to_master", "pub"),
        ("/master_com/master_to_ros/x4f", "sub"),  # MOTOR_GET_WHEELS
        ("/master_com/master_to_ros/x42", "sub"),  # MOTOR_GET_TEMPERATURE
    ],
    "laser_driver_node": [
        ("/master_com/master_to_ros/x5a", "sub"),  # LASER_DIST
        ("/arc/laser_distance", "pub"),
    ],
    "rfid_driver": [("/master_com/master_to_ros/x85", "sub")],  # ESP_TO_GS
    "arm_twist_driver": [("/master_com/ros_to_master", "pub")],
    "ueuos_driver": [("/master_com/ros_to_master", "pub")],
    "feed_driver": [("/master_com/ros_to_master", "pub")],
    "tunnel": [("/master_com/ros_to_master", "pub")],
    "autonomy_switch_spam": [("/master_com/ros_to_master", "pub")],
}

TUNNEL_SERVICES = [
    "/tunnel/reboot_pc",
    "/tunnel/arch_launch_stack",
    "/tunnel/arch_send_goal",
]
# Frame the tunnel node sends for /tunnel/arch_launch_stack:
# UNIQUE_TUNNEL_FORWARD_ID, ARCH_LAUNCH_STACK_ID.
ARCH_LAUNCH_STACK_FRAME = MasterMessage(cmd=MasterMessage.GS_TO_PC, data=[73, 47])

# Out-of-range wheel velocities (m/s) and the int8 bytes they must clamp to:
# 5.0 -> 127, -5.0 -> -127 (0x81), NaN -> 0, 0.5 -> 50. All angles are 0.
WHEEL_STATES = WheelStates(
    front_right=WheelState(velocity=5.0),
    back_right=WheelState(velocity=-5.0),
    back_left=WheelState(velocity=math.nan),
    front_left=WheelState(velocity=0.5),
)
WHEEL_FRAME_DATA = [127, 0x81, 0, 50, 0, 0, 0, 0]

# Out-of-range space mouse input and the bytes it must clamp to:
# -2.0 -> 0, 2.0 -> 200, NaN -> 100 (neutral), 0.5 -> 150, 0.0 -> 100.
ARM_TWIST = Twist()
ARM_TWIST.linear.x = -2.0
ARM_TWIST.linear.y = 2.0
ARM_TWIST.linear.z = math.nan
ARM_TWIST.angular.x = 0.5
ARM_TWIST_FRAME_DATA = [0, 200, 100, 150, 100, 100]


@pytest.mark.launch_test
@launch_testing.parametrize("mode", ["gs", "pc", "arm"])
def generate_test_description(mode):
    drivers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("kalman_drivers") / "launch/drivers.launch.py")
        ),
        launch_arguments={"mode": mode}.items(),
    )
    # arm mode starts no processes; the timer keeps the launch alive until the
    # tests are done.
    keep_alive = TimerAction(period=10 * TIMEOUT, actions=[])
    return (
        launch.LaunchDescription(
            [drivers, keep_alive, launch_testing.actions.ReadyToTest()]
        ),
        {},
    )


class TestDriversLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("drivers_launch_test")

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

    def frame_after_publishing(self, cmd, msg_type, topic, msg):
        """Publish msg on topic until a frame with cmd reaches the Master."""
        frames = []
        sub = self.node.create_subscription(
            MasterMessage,
            "master_com/ros_to_master",
            lambda frame: frames.append(frame) if frame.cmd == cmd else None,
            10,
        )
        pub = self.node.create_publisher(msg_type, topic, 10)
        try:
            self.spin_until(lambda: frames, every=lambda: pub.publish(msg))
            return list(frames[0].data) if frames else None
        finally:
            self.node.destroy_publisher(pub)
            self.node.destroy_subscription(sub)

    def launched_nodes(self):
        # Hidden nodes (e.g. a ros2 CLI daemon) start with an underscore.
        return {
            name
            for name in self.node.get_node_names()
            if name != self.node.get_name()
            and not name.startswith(("launch_ros_", "_"))
        }

    def test_starts_exactly_the_keep_drivers(self, mode):
        expected = KEEP_NODES[mode]
        self.spin_until(lambda: expected <= self.launched_nodes())
        # Give any extra node a chance to show up before comparing.
        self.spin_until(lambda: False, timeout=SETTLE)
        self.assertEqual(self.launched_nodes(), expected)

    def test_master_topics(self, mode):
        def endpoints(topic, kind):
            infos = (
                self.node.get_publishers_info_by_topic(topic)
                if kind == "pub"
                else self.node.get_subscriptions_info_by_topic(topic)
            )
            return {info.node_name for info in infos}

        expected = [
            (topic, node, kind)
            for node in KEEP_NODES[mode]
            for topic, kind in MASTER_TOPICS[node]
        ]
        self.spin_until(
            lambda: all(node in endpoints(topic, kind) for topic, node, kind in expected)
        )
        for topic, node, kind in expected:
            self.assertIn(node, endpoints(topic, kind), f"{node} {kind} {topic}")

    def test_tunnel_services(self, mode):
        if mode != "gs":
            self.skipTest("the tunnel node only runs in gs mode")

        def services():
            return dict(self.node.get_service_names_and_types())

        self.spin_until(lambda: all(name in services() for name in TUNNEL_SERVICES))
        for name in TUNNEL_SERVICES:
            self.assertEqual(services().get(name), ["std_srvs/srv/Empty"], name)

    def test_tunnel_sends_frame(self, mode):
        if mode != "gs":
            self.skipTest("the tunnel node only runs in gs mode")

        sent = []
        sub = self.node.create_subscription(
            MasterMessage,
            "master_com/ros_to_master",
            lambda msg: sent.append(msg) if msg.cmd == MasterMessage.GS_TO_PC else None,
            10,
        )
        client = self.node.create_client(Empty, "/tunnel/arch_launch_stack")
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT), "no tunnel service")
            future = client.call_async(Empty.Request())
            self.assertTrue(self.spin_until(future.done), "tunnel did not answer")
            self.assertTrue(self.spin_until(lambda: sent), "tunnel sent no GS_TO_PC frame")
            self.assertEqual(list(sent[0].data), list(ARCH_LAUNCH_STACK_FRAME.data))
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)

    def test_wheel_driver_clamps(self, mode):
        if mode == "arm":
            self.skipTest("wheel_driver does not run in arm mode")

        data = self.frame_after_publishing(
            MasterMessage.MOTOR_SET_WHEELS, WheelStates, "wheel_states", WHEEL_STATES
        )
        self.assertEqual(data, WHEEL_FRAME_DATA)

    def test_arm_twist_driver_clamps(self, mode):
        if mode != "gs":
            self.skipTest("arm_twist_driver only runs in gs mode")

        data = self.frame_after_publishing(
            MasterMessage.ARM_SEND_SPACEMOUSE, Twist, "spacenav/twist", ARM_TWIST
        )
        self.assertEqual(data, ARM_TWIST_FRAME_DATA)


@launch_testing.post_shutdown_test()
class TestDriversShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        if not proc_info.processes():
            self.skipTest("no nodes were started")
        # SIGINT on shutdown; every node must exit cleanly.
        launch_testing.asserts.assertExitCodes(proc_info)
