import time
import numpy as np
import math

from kalman_supervisor.state import State
from kalman_supervisor.modules import *

WORKING_FRAME = "map"
GATE_MARKER_ID = 297
APPROACH_DISTANCE = 1.0
PASS_DURATION = 6.0
PASS_SPEED = 0.3
SEARCH_ROTATION_SPEED = 0.3
MAX_ROTATE_TO_GATE_SPEED = 0.3


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def get_airlock_context(supervisor):
    if not hasattr(supervisor, "airlock_context"):
        supervisor.airlock_context = {}
    return supervisor.airlock_context


class RscpWaitBeforeAirlock(State):
    def __init__(self):
        super().__init__("rscp_wait_before_airlock")

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Airlock: waiting before search...")
        self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.ARMED)
        self.start_time = time.time()

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        if time.time() - self.start_time >= 2.0:
            return "rscp_airlock_search_gate"

        return None


class RscpAirlockSearchGate(State):
    def __init__(self):
        super().__init__("rscp_airlock_search_gate")

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Airlock: searching for gate...")
        self.supervisor.ueuos.set_rscp_state(Ueuos.RscpState.AUTONOMOUS)
        self.supervisor.aruco.enable()

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        gate = self.supervisor.aruco.gate_pos(GATE_MARKER_ID, WORKING_FRAME)
        if gate is None:
            self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, SEARCH_ROTATION_SPEED)
            return None

        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
        p1, p2 = gate
        self.supervisor.get_logger().info("[RSCP] Airlock: gate found.")

        p1_2d, p2_2d = p1[:2], p2[:2]
        M = (p1_2d + p2_2d) / 2.0
        v = p2_2d - p1_2d
        v_len = np.linalg.norm(v)

        if v_len < 0.1:
            self.supervisor.get_logger().warn("[RSCP] Airlock: gate width too small.")
            return None

        v_dir = v / v_len
        perp1 = np.array([-v_dir[1], v_dir[0]])
        perp2 = np.array([v_dir[1], -v_dir[0]])

        robot_pos = self.supervisor.tf.robot_pos(WORKING_FRAME)[:2]
        to_robot = robot_pos - M
        perp = perp1 if np.dot(perp1, to_robot) > 0 else perp2

        E = M + perp * APPROACH_DISTANCE
        get_airlock_context(self.supervisor)["entry_pos"] = E
        get_airlock_context(self.supervisor)["gate_mid"] = M

        # log tag positions, mid point and entry point
        self.supervisor.get_logger().info(
            f"[RSCP] Airlock: gate p1: {p1_2d}, p2: {p2_2d}, mid: {M}, entry: {E}"
        )

        return "rscp_airlock_navigate_entry"

    def exit(self) -> None:
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)


class RscpAirlockNavigateEntry(State):
    def __init__(self):
        super().__init__("rscp_airlock_navigate_entry")

    def enter(self) -> None:
        ctx = get_airlock_context(self.supervisor)
        E = ctx["entry_pos"]
        self.supervisor.get_logger().info(f"[RSCP] Airlock: navigating to {E}")
        self.supervisor.nav.send_goal(np.append(E, 0), WORKING_FRAME)

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        if not self.supervisor.nav.has_goal():
            self.supervisor.get_logger().info("[RSCP] Airlock: reached entry pos.")
            return "rscp_airlock_rotate_to_gate"

        return None

    def exit(self) -> None:
        if self.supervisor.nav.has_goal():
            self.supervisor.nav.cancel_goal()


class RscpAirlockRotateToGate(State):
    def __init__(self):
        super().__init__("rscp_airlock_rotate_to_gate")

    def enter(self) -> None:
        ctx = get_airlock_context(self.supervisor)

        gate = self.supervisor.aruco.gate_pos(GATE_MARKER_ID)
        if gate is not None:
            p1, p2 = gate
            M = (p1[:2] + p2[:2]) / 2.0
            ctx["gate_mid"] = M

        M = ctx.get("gate_mid")
        if M is not None:
            robot_pos = self.supervisor.tf.robot_pos(WORKING_FRAME)[:2]
            to_gate = M - robot_pos
            self.target_yaw = math.atan2(to_gate[1], to_gate[0])
        else:
            self.target_yaw = self.supervisor.tf.robot_rot_2d(WORKING_FRAME)
            self.supervisor.get_logger().warn(
                "[RSCP] Airlock: gate position unknown, using current orientation"
            )

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        yaw = self.supervisor.tf.robot_rot_2d(WORKING_FRAME)
        diff = normalize_angle(self.target_yaw - yaw)

        if abs(diff) < 0.1:
            self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
            return "rscp_airlock_drive_through"

        vel = 0.5 * np.sign(diff) + 0.1 * diff
        vel = max(-MAX_ROTATE_TO_GATE_SPEED, min(MAX_ROTATE_TO_GATE_SPEED, vel))
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, vel)
        return None

    def exit(self) -> None:
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)


class RscpAirlockDriveThrough(State):
    def __init__(self):
        super().__init__("rscp_airlock_drive_through")

    def enter(self) -> None:
        self.supervisor.get_logger().info("[RSCP] Airlock: driving through.")
        self.start_time = time.time()
        self.supervisor.aruco.disable()

    def tick(self) -> str | None:
        if not self.supervisor.rscp.is_armed():
            return "rscp_idle"

        if time.time() - self.start_time < PASS_DURATION:
            self.supervisor.cmd_vel.send_cmd_vel(PASS_SPEED, 0.0, 0.0)
            return None

        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
        self.supervisor.get_logger().info("[RSCP] Airlock completed.")
        return "rscp_idle"

    def exit(self) -> None:
        self.supervisor.cmd_vel.send_cmd_vel(0.0, 0.0, 0.0)
