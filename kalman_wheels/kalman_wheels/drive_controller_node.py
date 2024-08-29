import rclpy
import numpy as np
import time
from kalman_interfaces.msg import Drive, WheelStates
from rclpy.node import Node

def lerp(a, b, t):
    return a + (b - a) * t

def cos_from_sin(sin):
    return np.sqrt(1 - sin ** 2)

class State:
    pass

class SetupDriving(State):
    pass

class Driving(State):
    pass

class SetupRotating(State):
    pass

class Rotating(State):
    pass

class RotatingTimeout(State):
    def __init__(self, timeout):
        self.end_time = time.time() + timeout


class DriveController(Node):
    def __init__(self):
        super().__init__("drive_controller")

        self.rate = self.declare_parameter("rate", 30)
        self.timeout = self.declare_parameter("timeout", 1.0)
        self.max_speed = self.declare_parameter("max_speed", 1)
        self.max_rotation_speed = self.declare_parameter("max_rotation_speed", np.pi / 2)
        self.min_turn_radius = self.declare_parameter("min_turn_radius", 0.75)
        self.sideways_turn_radius_scale = self.declare_parameter("sideways_turn_radius_scale", 2.0)
        self.rotation_timeout = self.declare_parameter("rotation_timeout", 0.5)
        self.swivel_speed = self.declare_parameter("swivel_speed", np.pi / 2)
        self.motor_accel = self.declare_parameter("motor_accel", 4.0)
        self.max_wheel_angle = self.declare_parameter("max_wheel_angle", 110 * np.pi / 180) # larger than 90

        self.drive_sub = self.create_subscription(
            Drive, "drive", self.drive_cb, qos_profile=10
        )
        self.wheel_states_pub = self.create_publisher(
            WheelStates, "wheel_states", qos_profile=10
        )
        self.create_timer(1.0 / self.rate.value, self.tick)
        
        self.last_drive_msg = Drive()
        self.last_drive_msg_time = 0
        self.state: State = SetupDriving()
        self.target_angles = [0, 0, 0, 0]
        self.target_velocities = [0, 0, 0, 0]
        self.current_angles = [0, 0, 0, 0]
        self.current_velocities = [0, 0, 0, 0]
    

    def drive_cb(self, msg: Drive):
        self.last_drive_msg = msg
        self.last_drive_msg_time = time.time()

    def send_twist(self, x, y, angular, speed=1.0):
        # self.get_logger().info(f"Sending twist: x={x}, y={y}, angular={angular}, speed={speed}")
        
        TURN_VECTORS = [
            vec / np.linalg.norm(vec)
            for vec in [
                np.array([-wheel_pos[1], wheel_pos[0]])  # rot +90
                for wheel_pos in [
                    np.array([0.4, 0.33]),  # front left
                    np.array([0.4, -0.33]),  # front right
                    np.array([-0.4, 0.33]),  # back left
                    np.array([-0.4, -0.33]),  # back right
                ]
            ]
        ]

        # linear motion vector
        linear_vector = np.array([x, y])

        # angular motion vectors
        robot_radius = 0.5
        angular_vectors = [vec * angular * robot_radius for vec in TURN_VECTORS]

        # final wheel vectors
        wheel_vectors = [
            linear_vector + angular_vector for angular_vector in angular_vectors
        ]

        # wheel velocities
        wheel_velocities = [np.linalg.norm(vec) for vec in wheel_vectors]

        # wheel angles
        wheel_angles = [np.arctan2(vec[1], vec[0]) for vec in wheel_vectors]

        # Limit wheel velocities to 1 unit.
        max_wheel_vel = np.max(np.abs(wheel_velocities))
        if max_wheel_vel > 1.0:
            scale = 1.0 / max_wheel_vel
            wheel_velocities = [vel * scale for vel in wheel_velocities]

        # Scale wheel velocities by the speed.
        wheel_velocities = [v * speed for v in wheel_velocities]

        # Flip wheel velocities and angles if the absolute angle exceeds 110 degrees.
        # Also flip the velocity sign.
        # The input parameters were tuned to never make the wheels flip unexpectedly.
        # This is only useful when rotating in place, where turning radius is 0 <<< robot radius.
        def flip_angle(angle):
            angle += np.pi
            return np.arctan2(np.sin(angle), np.cos(angle))
        for i in range(len(wheel_angles)):
            if abs(wheel_angles[i]) > self.max_wheel_angle.value + 1e-3:
                wheel_angles[i] = flip_angle(wheel_angles[i])
                wheel_velocities[i] *= -1

        self.target_angles = wheel_angles
        self.target_velocities = wheel_velocities

        msg = WheelStates()
        msg.front_left.velocity = self.target_velocities[0]
        msg.front_left.angle = self.target_angles[0]
        msg.front_right.velocity = self.target_velocities[1]
        msg.front_right.angle = self.target_angles[1]
        msg.back_left.velocity = self.target_velocities[2]
        msg.back_left.angle = self.target_angles[2]
        msg.back_right.velocity = self.target_velocities[3]
        msg.back_right.angle = self.target_angles[3]
        self.wheel_states_pub.publish(msg)
    

    def tick(self):
        # Skip tick if no drive message has been received recently.
        if time.time() - self.last_drive_msg_time > self.timeout.value:
            return

        # Update current angles.
        dt = 1.0 / self.rate.value
        for i in range(len(self.target_angles)):
            dx = self.target_angles[i] - self.current_angles[i]
            dx = np.clip(dx, -self.swivel_speed.value * dt, self.swivel_speed.value * dt)
            self.current_angles[i] += dx

        # Update current velocities.
        for i in range(len(self.target_velocities)):
            dv = self.target_velocities[i] - self.current_velocities[i]
            dv = np.clip(dv, -self.motor_accel.value * dt, self.motor_accel.value * dt)
            self.current_velocities[i] += dv

        # Compute turn radius.
        msg = self.last_drive_msg
        forwards_inv_radius = np.clip(msg.inv_radius, -1.0 / self.min_turn_radius.value, 1.0 / self.min_turn_radius.value)
        sideways_inv_radius = forwards_inv_radius / self.sideways_turn_radius_scale.value
        speed = np.clip(msg.speed, -self.max_speed.value, self.max_speed.value)
        if abs(msg.sin_angle) <= 1:
            # regular driving
            inv_radius = lerp(forwards_inv_radius, sideways_inv_radius, np.abs(msg.sin_angle))

            x = cos_from_sin(msg.sin_angle)
            y = msg.sin_angle
            angular = inv_radius
        else:
            # over-translated driving
            sin_angle = (2 - abs(msg.sin_angle)) * np.sign(msg.sin_angle)
            angle = np.arctan2(sin_angle, -cos_from_sin(sin_angle))
            angle = np.clip(angle, -self.max_wheel_angle.value, self.max_wheel_angle.value)
            over_translate_factor = (abs(angle) - np.pi / 2) / (self.max_wheel_angle.value - np.pi / 2)

            inv_radius = lerp(sideways_inv_radius, 0, over_translate_factor)

            x = -cos_from_sin(sin_angle)
            y = sin_angle
            angular = inv_radius
        
        # rotation twist
        rotation_angular = np.clip(msg.rotation, -self.max_rotation_speed.value, self.max_rotation_speed.value)

        if isinstance(self.state, SetupDriving):
            self.send_twist(x, y, angular, 0.001)

            # Transition to driving if the wheels are aligned.
            if np.allclose(self.current_angles, self.target_angles, atol=0.1):
                self.state = Driving()

            # Transition to rotation if the rotation is non-zero.
            if msg.rotation != 0:
                self.state = SetupRotating()
        elif isinstance(self.state, Driving):
            self.send_twist(x, y, angular, speed)

            # Transition to rotation if the rotation is non-zero and the weheels are stopped.
            if msg.rotation != 0 and np.allclose(self.current_velocities, [0, 0, 0, 0], atol=0.1):
                self.state = SetupRotating()
        elif isinstance(self.state, SetupRotating):
            self.send_twist(0, 0, 0.001)

            # Transition to rotation once the wheels are aligned.
            if np.allclose(self.current_angles, self.target_angles, atol=0.1):
                self.state = Rotating()

            # Transition to driving if the rotation is zero.
            if msg.rotation == 0:
                self.state = SetupDriving()
        elif isinstance(self.state, Rotating):
            # Begin timeout if the rotation is zero.
            if msg.rotation == 0:
                self.state = RotatingTimeout(self.rotation_timeout.value)
            else:
                self.send_twist(0, 0, rotation_angular)
        elif isinstance(self.state, RotatingTimeout):
            self.send_twist(0, 0, 0.001)

            # Transition to driving once the timeout is over.
            if time.time() > self.state.end_time:
                self.state = SetupDriving()

            # Cancel timeout if rotation is non-zero.
            if msg.rotation != 0:
                self.state = Rotating()

def main():
    try:
        rclpy.init()
        node = DriveController()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
