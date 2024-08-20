import time
import warnings
import rclpy
import yaml
import os
import matplotlib.pyplot as plt
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, NavSatFix
from threading import Lock
from rclpy.callback_groups import ReentrantCallbackGroup

# input is wxyz
def quat_to_basis(q: np.ndarray) -> np.ndarray:
    return (
        np.array(
            [
                1 - 2 * (q[2] * q[2] + q[3] * q[3]),
                2 * (q[1] * q[2] + q[3] * q[0]),
                2 * (q[1] * q[3] - q[2] * q[0]),
                2 * (q[1] * q[2] - q[3] * q[0]),
                1 - 2 * (q[1] * q[1] + q[3] * q[3]),
                2 * (q[2] * q[3] + q[1] * q[0]),
                2 * (q[1] * q[3] + q[2] * q[0]),
                2 * (q[2] * q[3] - q[1] * q[0]),
                1 - 2 * (q[1] * q[1] + q[2] * q[2]),
            ]
        )
        .reshape(3, 3)
        .T
    )


def spherical_normal(lambda_: float, phi: float) -> np.ndarray:
    return np.array(
        [
            np.cos(phi) * np.cos(lambda_),
            np.cos(phi) * np.sin(lambda_),
            np.sin(phi),
        ]
    )


def great_circle_bearing(lat_lon1: np.ndarray, lat_lon2: np.ndarray) -> float:
    phi1 = lat_lon1[0] * np.pi / 180.0
    lambda1 = lat_lon1[1] * np.pi / 180.0
    phi2 = lat_lon2[0] * np.pi / 180.0
    lambda2 = lat_lon2[1] * np.pi / 180.0

    n1 = spherical_normal(lambda1, phi1)
    n2 = spherical_normal(lambda2, phi2)

    east1 = np.cross(np.array([0, 0, 1]), n1)
    north1 = np.cross(n1, east1)

    n_diff2 = np.dot(n2, n1) * n1
    azim_vec2 = (n2 - n_diff2) / np.linalg.norm(n2 - n_diff2)

    cos_alpha = np.dot(north1, azim_vec2)
    sin_alpha = np.dot(east1, azim_vec2)
    alpha = np.arctan2(sin_alpha, cos_alpha)
    return (alpha * 180.0 / np.pi).item()

def great_circle_distance(lat_lon1: np.ndarray, lat_lon2: np.ndarray) -> float:
    phi1 = lat_lon1[0] * np.pi / 180.0
    lambda1 = lat_lon1[1] * np.pi / 180.0
    phi2 = lat_lon2[0] * np.pi / 180.0
    lambda2 = lat_lon2[1] * np.pi / 180.0

    n1 = spherical_normal(lambda1, phi1)
    n2 = spherical_normal(lambda2, phi2)

    angle = np.arccos(np.dot(n1, n2))
    return (6371000.0 * angle).item()

def yaw_to_bearing(yaw: float) -> float:
    return -yaw * 180.0 / np.pi + 90.0

def bearing_to_yaw(bearing: float) -> float:
    return (-bearing + 90.0) * np.pi / 180.0

def linear_regression(x, y):
    A = np.vstack([x, np.ones(len(x))]).T
    B = y.reshape(-1, 1)
    X = np.linalg.pinv(A) @ B
    return (X[0], X[1])

# mean absolute deviation
def mad(data):
    return np.mean(np.abs(data - np.mean(data)))

CMD_VEL_RATE = 10  # Hz
SAMPLE_RATE = 4  # Hz
PRE_SAMPLE_DELAY = 1.0  # s


class DeclinationCalibration(Node):
    def __init__(self):
        super().__init__("declination_calibration")

        # Declare parameters.
        self.delay = self.declare_parameter("delay", 3.0).value
        self.duration = self.declare_parameter("duration", 30.0).value
        self.velocity = self.declare_parameter("velocity", 0.5).value
        self.imu_yaw = self.declare_parameter("imu_yaw", -np.pi / 2).value # physical IMU rotation

        # Init cmd_vel publisher.
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Init subscribers.
        self.callback_group = ReentrantCallbackGroup()
        self.imu_sub = self.create_subscription(
            Imu,
            "imu/data",
            self.imu_callback,
            10,
            callback_group=self.callback_group,
        )
        self.gps_sub = self.create_subscription(
            NavSatFix,
            "gps/fix",
            self.gps_callback,
            10,
            callback_group=self.callback_group,
        )

        # Latest messages.
        self.initial_lat_lon = None
        self.last_xy = None # x = meters east, y = meters north
        self.last_yaw = None
        self.sub_lock = Lock()

        # Begin a one-shot timer to start the calibration.
        self.timer = self.create_timer(self.delay, self.calibrate, callback_group=self.callback_group)

    def imu_callback(self, msg: Imu):
        quat = np.array(
            [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
        )
        basis = quat_to_basis(quat)
        # Yaw is the right-handed angle from the east.
        yaw = np.arctan2(basis[1, 0], basis[0, 0])
        yaw -= self.imu_yaw
        yaw = np.arctan2(np.sin(yaw), np.cos(yaw))
        with self.sub_lock:
            self.last_yaw = yaw.item()


    def gps_callback(self, msg: NavSatFix):
        with self.sub_lock:
            if self.initial_lat_lon is None:
                self.initial_lat_lon = (msg.latitude, msg.longitude)
                return

        bearing = great_circle_bearing(
            self.initial_lat_lon, (msg.latitude, msg.longitude)
        )
        distance = great_circle_distance(
            self.initial_lat_lon, (msg.latitude, msg.longitude)
        )
        yaw = bearing_to_yaw(bearing)

        with self.sub_lock:
            self.last_xy = np.array([
                distance * np.cos(yaw),
                distance * np.sin(yaw),
            ])


    def calibrate(self):
        # Cancel the one-shot timer.
        self.timer.cancel()
        self.get_logger().info("Starting calibration.")

        samples = []
        declination_error_history = [] # to calculate deviation

        # Start taking samples and simultaneously drive the robot forwards.
        last_sample_time = time.time() - 1.0 / SAMPLE_RATE + PRE_SAMPLE_DELAY
        warnings.filterwarnings("ignore") # silences matplotlib
        plt.ion()
        while True:
            if time.time() - last_sample_time > 1.0 / SAMPLE_RATE:
                last_sample_time = time.time()

                # Take a sample.
                with self.sub_lock:
                    samples.append((self.last_yaw, self.last_xy))

                # Re-calculate the average IMU bearing over all samples.
                avg_yaw = np.mean([sample[0] for sample in samples]).item()
                # And use the coords of the first and the last sample to calculate the GPS bearing.
                # Or even better, use linear regression to calculate the bearing.
                if len(samples) >= 2:
                    xy_yaw_first_last = np.arctan2(
                        samples[-1][1][1] - samples[0][1][1],
                        samples[-1][1][0] - samples[0][1][0],
                    )

                    x = np.array([sample[1][0] for sample in samples])
                    y = np.array([sample[1][1] for sample in samples])
                    a, _ = linear_regression(x, y)
                    xy_yaw_reg = np.arctan(a).item()

                    if np.abs(xy_yaw_first_last) < np.pi / 2:
                        xy_yaw = xy_yaw_reg
                    else:
                        xy_yaw = xy_yaw_reg + np.pi
                        xy_yaw = np.arctan2(np.sin(xy_yaw), np.cos(xy_yaw)).item()

                    # Calculate the declination.
                    # Actually, since the IMU might already account for the declination,
                    # we can at most have the error that must be added to the current declination.
                    declination_error = yaw_to_bearing(avg_yaw) - yaw_to_bearing(xy_yaw)
                    declination_error_history.append(declination_error)
                else:
                    xy_yaw = None

                # Log the sample.
                self.get_logger().info(
                    f"Taking sample #{len(samples)}:\n"
                    + f"yaw: {samples[-1][0]:.2f}\n"
                    + f"x, y: {samples[-1][1]}\n"
                    + f"Mean yaw: {avg_yaw:.2f} rad"
                    + (
                        f"\nXY yaw: {xy_yaw:.2f} rad\n"
                        + f"Declination error: {declination_error_history[-1]:.2f} deg +- {mad(declination_error_history):.2f} deg"
                        if xy_yaw is not None else ""
                    )
                )

                # Redraw the plot.
                # Each sample point is an arrow positioned at lat_long and pointing in the direction of yaw.
                plt.clf()
                plt.gca().set_aspect("equal", adjustable="box")
                if xy_yaw is not None:
                    plt.title(f"Declination error: {declination_error_history[-1]:.2f} deg +- {mad(declination_error_history):.2f} deg")
                for yaw, xy in samples:
                    plt.arrow(
                        xy[0],
                        xy[1],
                        np.cos(yaw) * 0.5,
                        np.sin(yaw) * 0.5,
                        width=0.05,
                        head_width=0.1,
                        head_length=0.1,
                    )
                a, b = linear_regression(
                    np.array([sample[1][0] for sample in samples]),
                    np.array([sample[1][1] for sample in samples]),
                )
                xs = np.linspace(plt.xlim()[0], plt.xlim()[1], 2)
                plt.plot(xs, a * xs + b, "r")
                plt.pause(0.01)
                plt.show()
                plt.pause(0.01)
                plt.show()
                plt.pause(0.01)
                plt.show()

                # Stop taking samples after the duration has passed.
                if len(samples) >= self.duration * SAMPLE_RATE:
                    msg = Twist()
                    self.cmd_vel_pub.publish(msg)
                    break

            # Drive the robot forwards.
            msg = Twist()
            msg.linear.x = self.velocity
            self.cmd_vel_pub.publish(msg)
            time.sleep(1.0 / CMD_VEL_RATE)

        # Read the current declination from config.
        config_path = os.path.join(
            os.path.expanduser("~"),
            ".config/kalman/imu_filter_madgwick_calibrated_declination.yaml",
        )
        old_declination = 0
        if os.path.exists(config_path):
            with open(config_path, "r") as f:
                config = yaml.safe_load(f)
                old_declination = (
                    config["/**"]["ros__parameters"]["declination"] * 180.0 / np.pi
                )  # the node expects radians
        new_declination = old_declination + declination_error
        if type(new_declination) is not float:
            new_declination = new_declination.item()

        # Save the coefficients to a file.
        os.makedirs(os.path.expanduser("~/.config/kalman"), exist_ok=True)
        with open(
            os.path.join(
                os.path.expanduser("~"),
                ".config/kalman/imu_filter_madgwick_calibrated_declination.yaml",
            ),
            "w",
        ) as f:
            yaml.dump(
                {
                    "/**": {
                        "ros__parameters": {
                            "declination": new_declination * np.pi / 180.0,  # the node expects radians
                        }
                    }
                },
                f,
            )

        # Print the coefficients.
        self.get_logger().info(
            f"Calibration is done. Declination was changed from {old_declination} to {new_declination} degrees."
        )

        # Show the plot in a blocking way before exiting.
        plt.ioff()
        plt.show()
        os._exit(0)

def main():
    try:
        rclpy.init()
        executor = MultiThreadedExecutor(num_threads=4)

        node = DeclinationCalibration()
        executor.add_node(node)
        executor.spin()
        node.destroy_node()

        executor.shutdown()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
