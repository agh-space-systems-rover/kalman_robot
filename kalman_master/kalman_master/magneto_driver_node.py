from matplotlib import animation, pyplot as plt
import rclpy
import time
import struct
from datetime import datetime as dt

import numpy as np
from scipy.optimize import curve_fit
import pandas as pd

from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray

from kalman_interfaces.msg import MagnetoData
from kalman_interfaces.srv import RequestMagnetoTare

import threading


class MagnetoDriver(Node):
    def __init__(self):
        super().__init__("magneto_driver")

        self.LOG_DIR = self.declare_parameter(
            "log_dir", "/home/ros/Documents/ilmenite"
        ).value
        self.CALIBRATION_DIR = self.declare_parameter(
            "calibration_dir", "/home/ros/Documents/ilmenite/calib_data"
        ).value
        self.REQ_PERIOD = self.declare_parameter("request_period", 2.0).value

        # Init magneto listener
        self.magneto = self.create_subscription(
            UInt8MultiArray, "magneto/data", self.data_response, 10
        )

        # Init magneto data publisher
        self.magneto_data_pub = self.create_publisher(MagnetoData, "magneto/result", 10)

        self.magnet_req_pub = self.create_publisher(
            UInt8MultiArray, "magneto/request", 10
        )

        self.create_timer(self.REQ_PERIOD, self.request_data)

        # Init magneto tare service
        self.magneto_tare_service = self.create_service(
            RequestMagnetoTare, "magneto/request_tare", self.tare_request
        )

        ilmenit_data_points = {}
        percents = [0, 5, 10, 15, 25]

        for percent in percents:
            ilmenit_data_points[percent] = pd.read_csv(
                f"{self.CALIBRATION_DIR}/ilmenit_{percent}.csv"
            )
            ilmenit_data_points[percent]["radius"] = np.sqrt(
                np.sum(np.power(ilmenit_data_points[percent], 2), axis=1)
            )

        means = []
        for percent in percents:
            means.append(np.mean(ilmenit_data_points[percent]["radius"][-20:]))

        self._interpolation_coeffs, covariance = curve_fit(
            self._fit_percents, means, percents
        )

        self._timestamp_start = dt.now()

        self._filename = f"{self.LOG_DIR}/log_{self._timestamp_start.strftime('%d_%m_%y__%H_%M_%S')}.csv"

        with open(self._filename, "w") as result_file:
            result_file.write("timestamp;x;y;z;length;tared_length;percentage\n")

        self._last_vector = (0, 0, 0)
        self._last_length = 0

        self._tare_length = 0

        self.thread = threading.Thread(target=self.start_chart_animation)
        self.thread.start()

    def data_response(self, msg: UInt8MultiArray):
        x, y, z = struct.unpack("<fff", bytearray(msg.data))
        self.get_logger().info(f"Received: {x} {y} {z}")

        vector_length = np.sqrt(x**2 + y**2 + z**2)
        self._last_vector = (x, y, z)
        self._last_length = vector_length

        tared_length = vector_length - self._tare_length
        estimator = np.clip(
            self._fit_percents(tared_length, *self._interpolation_coeffs), 0, 100
        )

        msg = MagnetoData(
            x=x,
            y=y,
            z=z,
            length=vector_length,
            tared_length=tared_length,
            percentage=estimator,
        )

        delta_t = dt.now() - self._timestamp_start
        with open(self._filename, "a") as result_file:
            result_file.write(
                f"{delta_t.seconds};{x};{y};{z};{vector_length};{tared_length};{estimator}\n"
            )

        self.magneto_data_pub.publish(msg)

    def tare_request(
        self, request: RequestMagnetoTare.Request, response: RequestMagnetoTare.Response
    ):
        match (request.type):
            case RequestMagnetoTare.Request.NEW:
                self._tare_length = self._last_length
            case RequestMagnetoTare.Request.RESET:
                self._tare_length = 0.0

        self.get_logger().info(f"Tare request: {request.type}")

        return response

    @staticmethod
    def _fit_percents(x, a, b, c):
        return a * np.exp(b * x) + c

    def start_chart_animation(self):
        fig, ax = plt.subplots()

        def update(frame):
            df = pd.read_csv(self._filename, delimiter=";")

            if df.size == 0:
                self.get_logger().warn("No magneto data to display yet")
                return

            try:
                ax.clear()
                ax.plot(
                    df["timestamp"].values[-150:],
                    df["length"].values[-150:],
                    label="Vector magnitude",
                )
                ax.plot(
                    df["timestamp"].values[-150:],
                    df["tared_length"].values[-150:],
                    label="Tared vector magnitude",
                    color='green'
                )
                ax.set_title("Magnetic field vector length")
                ax.set_xlabel("Time [s]")
                ax.set_ylabel("Magnitude [uT]")

                ax.legend()
            except KeyError:
                self.get_logger().warn("Something wrong with csv keys")

        anim = animation.FuncAnimation(fig, update, interval=2000)

        plt.show()

    def request_data(self):
        msg = UInt8MultiArray()
        for i in range(3):
            self.magnet_req_pub.publish(msg)

def plot_from_file(filename):
    fig, ax = plt.subplots()

    df = pd.read_csv(filename, delimiter=";")

    if df.size == 0:
        print("No magneto data to display yet")
        return

    try:
        ax.clear()
        ax.plot(
            df["timestamp"].values[-150:],
            df["length"].values[-150:],
            label="Vector magnitude",
        )
        ax.plot(
            df["timestamp"].values[-150:],
            df["tared_length"].values[-150:],
            label="Tared vector magnitude",
            color='green'
        )
        ax.set_title("Magnetic field vector length")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Magnitude [uT]")

        ax.legend()
    except KeyError:
        print("Something wrong with csv keys")

    fig.show()
    plt.show()


def main():
    try:
        rclpy.init()
        node = MagnetoDriver()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
