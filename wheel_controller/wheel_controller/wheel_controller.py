import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from kalman_interfaces.msg import WheelsCommand, WheelStates
from enum import Enum
import numpy as np


class DrivingMode(Enum):
    NORMAL = 0
    IN_PLACE = 1
    SIDEWAYS = 2


class GSMovementControl:
    velocity: float
    i_radius: float
    translate: float


class WheelContoller(Node):
    def __init__(self):
        super().__init__("wheel_controller")
        self.wheel_state_pub = self.create_publisher(
            WheelStates, "/wheel_states", qos_profile=10
        )
        self.wheel_command_sub = self.create_subscription(
            WheelsCommand,
            "/station/wheels/command",
            self.handle_command,
            qos_profile=10,
        )
        self.length = 0.92
        self.width = 0.708
        self.wheel_radius = 0.15

        self.wh_angle = np.deg2rad(52)
        self.fl = -self.wh_angle
        self.fr = self.wh_angle
        self.bl = self.wh_angle
        self.br = -self.wh_angle

    def drive(self, fl, fr, bl, br, lvel, rvel, blvel=None, brvel=None):
        wheel_states = WheelStates()

        angle_multiplier = 1.0
        velocity_multiplier = 1.0

        if blvel and brvel:
            wheel_states.front_left.angle = float(fl) * angle_multiplier
            wheel_states.front_right.angle = float(fr) * angle_multiplier
            wheel_states.back_left.angle = -float(bl) * angle_multiplier
            wheel_states.back_right.angle = -float(br) * angle_multiplier

            wheel_states.front_left.velocity = float(lvel) * velocity_multiplier
            wheel_states.front_right.velocity = float(rvel) * velocity_multiplier
            wheel_states.back_left.velocity = float(blvel) * velocity_multiplier
            wheel_states.back_right.velocity = float(brvel) * velocity_multiplier
        else:
            wheel_states.front_left.angle = float(fl) * angle_multiplier
            wheel_states.front_right.angle = float(fr) * angle_multiplier
            wheel_states.back_left.angle = -float(bl) * angle_multiplier
            wheel_states.back_right.angle = -float(br) * angle_multiplier

            wheel_states.front_left.velocity = float(lvel) * velocity_multiplier
            wheel_states.front_right.velocity = float(rvel) * velocity_multiplier
            wheel_states.back_left.velocity = float(lvel) * velocity_multiplier
            wheel_states.back_right.velocity = float(rvel) * velocity_multiplier

        self.wheel_state_pub.publish(wheel_states)

    def handle_command(self, message: WheelsCommand):
        mode = message.mode

        if mode == DrivingMode.NORMAL.value:
            msg = GSMovementControl()
            msg.velocity = message.x
            msg.translate = message.y
            msg.i_radius = message.z

            twist = Twist()
            twist.linear.x = message.x
            twist.linear.y = message.y
            twist.angular.z = message.z

            self.driveGSCmdVel(msg)

        elif mode == DrivingMode.IN_PLACE.value:
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = message.z

            self.driveGSCmdVelInPlace(msg)

        elif mode == DrivingMode.SIDEWAYS.value:
            msg = GSMovementControl()
            msg.velocity = message.x
            msg.translate = message.y
            msg.i_radius = message.z
            self.driveGSCmdVelSideways(msg)

        else:
            self.get_logger().error("Invalid driving mode received")

    def driveGSCmdVel(self, msg: GSMovementControl):
        # Do nothing if rotate_lock is set to True - this means that we are in rotate_in_place mode
        # if not self.mode == MODE_ARC_CONTROL:
        #     return

        MAX_ANGLE = np.deg2rad(100)

        self.turn_in_place_start_time = None

        fl, bl, fr, br, l_vel, r_vel = (
            0.0,
            0.0,
            0.0,
            0.0,
            msg.velocity,
            msg.velocity,
        )
        if msg.i_radius:
            radius, velocity = -1.0 / (msg.i_radius), msg.velocity  # FIXME

            small_constant = np.hypot(
                np.abs(radius) - self.width / 2.0, self.length / 2.0
            )
            big_constant = np.hypot(
                np.abs(radius) + self.width / 2.0, self.length / 2.0
            )

            inside = np.arcsin(self.length / (2.0 * small_constant))

            outside = np.arcsin(self.length / (2.0 * big_constant))

            omega_inside = velocity / self.wheel_radius * small_constant

            omega_outside = velocity / self.wheel_radius * big_constant

            if radius > 0:
                fr, br, fl, bl = -inside, -inside, -outside, -outside
                if omega_inside and omega_outside:
                    l_vel = velocity
                    r_vel = l_vel * omega_inside / omega_outside
            else:
                fr, br, fl, bl = outside, outside, inside, inside
                if omega_inside and omega_outside:
                    r_vel = velocity
                    l_vel = r_vel * omega_inside / omega_outside

        l_vel, r_vel = np.clip(l_vel, -1, 1), np.clip(r_vel, -1, 1)
        fl, fr, bl, br = (
            fl + msg.translate*1.75,
            fr + msg.translate*1.75,
            bl - msg.translate*1.75,
            br - msg.translate*1.75,
        )
        fl, fr, bl, br = (
            np.clip(fl, -MAX_ANGLE, MAX_ANGLE),
            np.clip(fr, -MAX_ANGLE, MAX_ANGLE),
            np.clip(bl, -MAX_ANGLE, MAX_ANGLE),
            np.clip(br, -MAX_ANGLE, MAX_ANGLE),
        )

        # angle_multiplier = 1.0

        # fl *= angle_multiplier
        # fr *= angle_multiplier
        # bl *= angle_multiplier
        # br *= angle_multiplier

        self.drive(fl=fl, fr=fr, bl=bl, br=br, lvel=l_vel, rvel=r_vel)

    def driveGSCmdVelInPlace(self, msg: Twist):
        fr, fl, br, bl = self.fr, self.fl, self.br, self.bl

        vel = msg.angular.z * np.hypot(self.width, self.length) / 2.0
        vel = np.clip(vel, -1, 1)

        self.drive(
            fr=fr, fl=fl, br=-br, bl=-bl, lvel=vel, rvel=-vel, blvel=vel, brvel=-vel
        )

    def driveGSCmdVelSideways(self, msg: GSMovementControl):
        # if not self.mode == MODE_ARC_CONTROL:
        #     return

        MAX_TRANSLATE_ANGLE = 10
        MIN_ANGLE = -np.deg2rad(100)
        MAX_ANGLE = np.deg2rad(100)

        max_delta = min(abs(MIN_ANGLE), abs(MAX_ANGLE)) - np.deg2rad(90)
        min_radius = (self.width / np.tan(max_delta) + self.length) / 2.0

        self.turn_in_place_start_time = None

        velocity = msg.velocity

        fr = np.deg2rad(90)
        bl = np.deg2rad(90)
        fl = -np.deg2rad(90)
        br = -np.deg2rad(90)

        front_vel = velocity
        back_vel = velocity

        if msg.i_radius:
            radius = np.clip(-1.0 / (msg.i_radius), -min_radius, min_radius)

            alfa = np.arctan(self.width / (2 * radius - self.length))
            beta = np.arctan(self.width / (2 * radius + self.length))

            fr += alfa
            fl -= alfa
            br += beta
            bl -= beta

            if velocity != 0:
                omega_inside = (
                    velocity
                    / self.wheel_radius
                    * np.hypot(abs(radius) - self.length / 2, self.width / 2)
                )

                omega_outside = (
                    velocity
                    / self.wheel_radius
                    * np.hypot(abs(radius) + self.length / 2, self.width / 2)
                )

                outside_vel = velocity
                inside_vel = outside_vel * omega_inside / omega_outside

                if radius > 0:
                    front_vel = inside_vel
                    back_vel = outside_vel
                else:
                    front_vel = outside_vel
                    back_vel = inside_vel

        front_vel = np.clip(front_vel, -1, 1)
        back_vel = np.clip(back_vel, -1, 1)

        t_rad = np.deg2rad(msg.translate * MAX_TRANSLATE_ANGLE)
        def clip(v: float) -> float:
            return np.clip(v, MIN_ANGLE, MAX_ANGLE)

        fl = clip(fl + t_rad)
        fr = clip(fr + t_rad)
        bl = clip(bl + t_rad)
        br = clip(br + t_rad)

        self.drive(
            fl,
            fr,
            -bl,
            -br,
            lvel=-front_vel,
            rvel=front_vel,
            brvel=-back_vel,
            blvel=back_vel,
        )


def main(args=None):
    rclpy.init(args=args)

    wheel_controller = WheelContoller()

    rclpy.spin(wheel_controller)

    wheel_controller.destroy_node()
    rclpy.shutdown()
