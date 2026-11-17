"""End-to-end test of ros_link over master_loopback.

Starts master_loopback and both sides of ros_link (rover and station) in a single
ROS graph, using the real production configs installed with the package. With
loopback_mangling enabled, every name forwarded to the other side gets a
"/loopback" suffix, so a test node can play both ends of the radio link:

    test node -> imu/data -> ros_link(rover) -> master_com/ros_to_master
        -> master_loopback -> master_com/master_to_ros/0x81
        -> ros_link(station) -> imu/data/loopback -> test node

This checks that the package installs, loads its configs, imports every
interface named in them, and forwards topics, services and actions.

Runs:
- "pc" and "arm" over a lossless link;
- "pc" over a link that drops 50% of frames. This exercises the ACK/retry logic
  of services and actions (retried at retry_rate, 1 Hz by default) and checks
  that duplicates are filtered so each call reaches its server exactly once.
"""

import threading
import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from ament_index_python import get_package_share_path
from geographic_msgs.msg import GeoPoint
from rclpy.action import ActionClient, ActionServer
from rclpy.action.client import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Int8

from kalman_interfaces.action import SupervisorGpsGoal
from kalman_interfaces.msg import ArmState
from kalman_interfaces.srv import SpoofGps

PACKAGE = "kalman_master"
TIMEOUT = 20.0  # seconds to wait for anything to cross a lossless link
# With 50% loss a frame and its ACK both survive 1 in 4 tries at 1 retry/s,
# and an action round trip needs several such exchanges.
LOSSY_TIMEOUT = 120.0


def timeout_for(loss_rate):
    return TIMEOUT if loss_rate == 0.0 else LOSSY_TIMEOUT


@pytest.mark.launch_test
@launch_testing.parametrize(
    "endpoint, loss_rate", [("pc", 0.0), ("arm", 0.0), ("pc", 0.5)]
)
def generate_test_description(endpoint, loss_rate):
    config = str(get_package_share_path(PACKAGE) / f"config/{endpoint}_ros_link.yaml")

    def ros_link(side):
        return launch_ros.actions.Node(
            package=PACKAGE,
            executable="ros_link",
            name=f"ros_link_{side}",
            parameters=[
                {
                    "config_path": config,
                    "side": side,
                    "rover_endpoint": endpoint,
                    "loopback_mangling": True,
                }
            ],
            output="screen",
        )

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package=PACKAGE,
                executable="master_loopback",
                parameters=[
                    {"loss_rate": loss_rate, "min_delay": 0.0, "max_delay": 0.01}
                ],
                output="screen",
            ),
            ros_link("rover"),
            ros_link("station"),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestRosLinkLoopback(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("ros_link_loopback_test")
        cls.group = ReentrantCallbackGroup()
        cls.executor = MultiThreadedExecutor(num_threads=8)
        cls.executor.add_node(cls.node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.node.destroy_node()
        rclpy.shutdown()

    # --- helpers ---------------------------------------------------------

    def wait_for(self, predicate, timeout=TIMEOUT, every=None):
        """Poll predicate() until true; call every() on each iteration (e.g. to republish)."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if every is not None:
                every()
            if predicate():
                return True
            time.sleep(0.1)
        return False

    def forward_topic(self, msg_type, name, msg, timeout=TIMEOUT):
        """Publish msg on `name` until it arrives on `name/loopback`; return the received copy."""
        received = []
        sub = self.node.create_subscription(
            msg_type, f"{name}/loopback", received.append, 10, callback_group=self.group
        )
        pub = self.node.create_publisher(msg_type, name, 10)
        try:
            ok = self.wait_for(
                lambda: received, timeout=timeout, every=lambda: pub.publish(msg)
            )
            self.assertTrue(ok, f"no message arrived on {name}/loopback")
            return received[0]
        finally:
            self.node.destroy_subscription(sub)
            self.node.destroy_publisher(pub)

    def wait_future(self, future, what, timeout=TIMEOUT):
        self.assertTrue(
            self.wait_for(future.done, timeout=timeout), f"timed out waiting for {what}"
        )
        return future.result()

    # --- common ----------------------------------------------------------

    def test_nodes_running(self, endpoint):
        expected = {"master_loopback", "ros_link_rover", "ros_link_station"}
        ok = self.wait_for(lambda: expected <= set(self.node.get_node_names()))
        self.assertTrue(
            ok, f"missing nodes: {expected - set(self.node.get_node_names())}"
        )

    # --- pc <-> gs -------------------------------------------------------

    def test_pc_topic_imu(self, endpoint, loss_rate):
        if endpoint != "pc":
            self.skipTest("pc link only")
        msg = Imu()
        msg.orientation.x, msg.orientation.y = 0.1, -0.2
        msg.orientation.z, msg.orientation.w = 0.3, 0.9
        out = self.forward_topic(Imu, "imu/data", msg, timeout_for(loss_rate))
        # Quantized to 1 byte over [-1, 1]: precision ~0.008.
        for axis in "xyzw":
            self.assertAlmostEqual(
                getattr(out.orientation, axis), getattr(msg.orientation, axis), delta=0.01
            )

    def test_pc_topic_gps(self, endpoint, loss_rate):
        if endpoint != "pc":
            self.skipTest("pc link only")
        msg = NavSatFix(latitude=50.0647, longitude=19.9450)
        msg.status.status = 0
        out = self.forward_topic(NavSatFix, "gps/fix", msg, timeout_for(loss_rate))
        self.assertAlmostEqual(out.latitude, msg.latitude, delta=1e-6)
        self.assertAlmostEqual(out.longitude, msg.longitude, delta=1e-6)
        self.assertEqual(out.status.status, 0)

    def test_pc_service_spoof_gps(self, endpoint, loss_rate):
        if endpoint != "pc":
            self.skipTest("pc link only")
        requests = []

        def serve(req, res):
            requests.append(req)
            return res

        # The rover side serves spoof_gps; the station side exposes spoof_gps/loopback.
        srv = self.node.create_service(
            SpoofGps, "spoof_gps", serve, callback_group=self.group
        )
        client = self.node.create_client(
            SpoofGps, "spoof_gps/loopback", callback_group=self.group
        )
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT))
            req = SpoofGps.Request(
                location=GeoPoint(latitude=50.0647, longitude=19.9450)
            )
            self.wait_future(
                client.call_async(req), "spoof_gps response", timeout_for(loss_rate)
            )
            self.assertEqual(len(requests), 1, "service should be called exactly once")
            self.assertAlmostEqual(requests[0].location.latitude, 50.0647, delta=1e-6)
            self.assertAlmostEqual(requests[0].location.longitude, 19.9450, delta=1e-6)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_service(srv)

    def test_pc_action_gps_goal(self, endpoint, loss_rate):
        if endpoint != "pc":
            self.skipTest("pc link only")
        goals = []

        def execute(goal_handle):
            goals.append(goal_handle.request)
            # max_feedback_rate is 1 Hz, so publish for a bit over a second.
            for _ in range(15):
                goal_handle.publish_feedback(SupervisorGpsGoal.Feedback(state="travel"))
                time.sleep(0.1)
            goal_handle.succeed()
            return SupervisorGpsGoal.Result()

        server = ActionServer(
            self.node,
            SupervisorGpsGoal,
            "supervisor/gps_goal",
            execute,
            callback_group=self.group,
        )
        client = ActionClient(
            self.node,
            SupervisorGpsGoal,
            "supervisor/gps_goal/loopback",
            callback_group=self.group,
        )
        feedback = []
        try:
            self.assertTrue(client.wait_for_server(timeout_sec=TIMEOUT))
            goal = SupervisorGpsGoal.Goal(
                location=GeoPoint(latitude=50.0647, longitude=19.9450)
            )
            goal_handle = self.wait_future(
                client.send_goal_async(
                    goal, feedback_callback=lambda fb: feedback.append(fb.feedback)
                ),
                "goal acceptance",
            )
            self.assertTrue(goal_handle.accepted, "goal was rejected")
            result = self.wait_future(
                goal_handle.get_result_async(), "action result", timeout_for(loss_rate)
            )
            self.assertEqual(result.status, GoalStatus.STATUS_SUCCEEDED)
            self.assertEqual(len(goals), 1, "goal should be executed exactly once")
            self.assertAlmostEqual(goals[0].location.latitude, 50.0647, delta=1e-6)
            self.assertAlmostEqual(goals[0].location.longitude, 19.9450, delta=1e-6)
            # Feedback is sent once without ACK, so it may be dropped on a lossy link.
            if loss_rate == 0.0:
                self.assertTrue(feedback, "no feedback crossed the link")
            for fb in feedback:
                self.assertEqual(fb.state, "travel")
        finally:
            client.destroy()
            server.destroy()

    # --- arm <-> gs ------------------------------------------------------

    def test_arm_topic_arm_state(self, endpoint):
        if endpoint != "arm":
            self.skipTest("arm link only")
        msg = ArmState(
            joint_1=1.0, joint_2=-0.5, joint_3=0.25, joint_4=-2.0, joint_5=1.5, joint_6=-3.0
        )
        out = self.forward_topic(ArmState, "arm_state", msg)
        # Coarsest quantization in the config is 1 byte over [-3.5, 3.5]: ~0.028.
        for i in range(1, 7):
            field = f"joint_{i}"
            self.assertAlmostEqual(
                getattr(out, field), getattr(msg, field), delta=0.03, msg=field
            )

    def test_arm_topic_gripper_station_to_rover(self, endpoint):
        if endpoint != "arm":
            self.skipTest("arm link only")
        out = self.forward_topic(Int8, "gripper/command_incremental", Int8(data=-5))
        self.assertEqual(out.data, -5)
