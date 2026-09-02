import os
import struct
import sys

import rclpy
import yaml
from PyQt5 import QtCore, QtWidgets
from kalman_interfaces.msg import MasterMessage
from rclpy.node import Node
from std_srvs.srv import Trigger


CALIB_PATH = os.path.expanduser("~/.config/kalman/storage_calib.yaml")


def calculate_calibration(known_1, measured_1, known_2, measured_2):
    if measured_1 == measured_2:
        raise ValueError("Measured values must be different.")

    scale = (known_2 - known_1) / (measured_2 - measured_1)
    bias = known_1 - scale * measured_1
    return scale, bias


def load_calibrations():
    if not os.path.exists(CALIB_PATH):
        return []
    with open(CALIB_PATH, "r") as file:
        return yaml.safe_load(file) or []


def save_calibration(board, channel, scale, bias):
    calibrations = [
        calibration
        for calibration in load_calibrations()
        if (calibration["board"], calibration["channel"]) != (board, channel)
    ]
    calibrations.append(
        {
            "board": board,
            "channel": channel,
            "scale": float(scale),
            "bias": float(bias),
        }
    )
    calibrations.sort(
        key=lambda calibration: (
            calibration["board"],
            calibration["channel"],
        )
    )

    os.makedirs(os.path.dirname(CALIB_PATH), exist_ok=True)
    with open(CALIB_PATH, "w") as file:
        yaml.safe_dump(calibrations, file, sort_keys=False)


class StorageCalibNode(Node):
    def __init__(self, measurement_callback):
        super().__init__("storage_calib")
        self.measurement_callback = measurement_callback
        self.request_pub = self.create_publisher(
            MasterMessage, "master_com/ros_to_master", 10
        )
        self.response_sub = self.create_subscription(
            MasterMessage,
            f"master_com/master_to_ros/{hex(MasterMessage.SCALE_RES)[1:]}",
            self.handle_response,
            10,
        )
        self.clear_client = self.create_client(
            Trigger, "science/storage/calibration/clear"
        )

    def request_measurement(self, board, channel):
        request = MasterMessage()
        request.cmd = MasterMessage.SCALE_REQ
        request.data = struct.pack("BB", board, channel)
        self.request_pub.publish(request)

    def handle_response(self, message):
        if len(message.data) < 6:
            return
        board, channel, value = struct.unpack("<BBi", bytes(message.data[:6]))
        self.measurement_callback(board, channel, value)

    def clear_driver_calibration(self):
        if self.clear_client.service_is_ready():
            self.clear_client.call_async(Trigger.Request())


class StorageCalibApp(QtWidgets.QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.measurement_callback = self.handle_measurement
        self.pending_measurement = None
        self.setWindowTitle("Storage Calibration")

        layout = QtWidgets.QVBoxLayout(self)

        sensor_group = QtWidgets.QGroupBox("Sensor")
        sensor_layout = QtWidgets.QFormLayout(sensor_group)
        self.board_input = QtWidgets.QSpinBox()
        self.board_input.setRange(0, 255)
        self.channel_input = QtWidgets.QSpinBox()
        self.channel_input.setRange(0, 255)
        sensor_layout.addRow("Board:", self.board_input)
        sensor_layout.addRow("Channel:", self.channel_input)
        layout.addWidget(sensor_group)

        self.known_inputs = []
        self.measured_inputs = []
        for number in range(1, 3):
            group = QtWidgets.QGroupBox(f"Measurement {number}")
            form = QtWidgets.QFormLayout(group)

            known_input = self.make_value_input()
            known_input.setSuffix(" g")
            measured_input = self.make_value_input()
            form.addRow("Known weight:", known_input)
            form.addRow("Measured/raw value:", measured_input)
            measure_button = QtWidgets.QPushButton("Measure")
            measure_button.clicked.connect(
                lambda _, index=number - 1: self.request_measurement(index)
            )
            form.addRow("", measure_button)
            layout.addWidget(group)

            self.known_inputs.append(known_input)
            self.measured_inputs.append(measured_input)

        self.known_inputs[1].setValue(100.0)

        self.result_label = QtWidgets.QLabel("Scale: ---\nBias: ---")
        layout.addWidget(self.result_label)

        self.measurement_status = QtWidgets.QLabel()
        layout.addWidget(self.measurement_status)

        self.save_button = QtWidgets.QPushButton("Calculate and save")
        self.save_button.clicked.connect(self.calculate_and_save)
        layout.addWidget(self.save_button)

        self.existing_label = QtWidgets.QLabel()
        layout.addWidget(self.existing_label)

        self.clear_button = QtWidgets.QPushButton("Clear entire config")
        self.clear_button.clicked.connect(self.clear_config)
        layout.addWidget(self.clear_button)

        self.board_input.valueChanged.connect(self.update_existing_label)
        self.channel_input.valueChanged.connect(self.update_existing_label)
        self.update_existing_label()
        self.setFixedWidth(420)

    @staticmethod
    def make_value_input():
        value_input = QtWidgets.QDoubleSpinBox()
        value_input.setRange(-1_000_000_000.0, 1_000_000_000.0)
        value_input.setDecimals(6)
        return value_input

    def calculate_and_save(self):
        try:
            scale, bias = calculate_calibration(
                self.known_inputs[0].value(),
                self.measured_inputs[0].value(),
                self.known_inputs[1].value(),
                self.measured_inputs[1].value(),
            )
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, "Cannot calibrate", str(error))
            return

        board = self.board_input.value()
        channel = self.channel_input.value()
        save_calibration(board, channel, scale, bias)
        self.result_label.setText(f"Scale: {scale:.9g}\nBias: {bias:.9g}")
        self.update_existing_label()
        QtWidgets.QMessageBox.information(
            self,
            "Calibration saved",
            f"Saved board {board}, channel {channel} to:\n{CALIB_PATH}",
        )

    def request_measurement(self, index):
        self.pending_measurement = index
        board = self.board_input.value()
        channel = self.channel_input.value()
        self.measurement_status.setText(
            f"Waiting for board {board}, channel {channel}..."
        )
        self.ros_node.request_measurement(board, channel)

    def handle_measurement(self, board, channel, value):
        if self.pending_measurement is None:
            return
        if (board, channel) != (
            self.board_input.value(),
            self.channel_input.value(),
        ):
            return

        self.measured_inputs[self.pending_measurement].setValue(float(value))
        self.measurement_status.setText(
            f"Received raw value {value} from "
            f"board {board}, channel {channel}."
        )
        self.pending_measurement = None

    def update_existing_label(self):
        board = self.board_input.value()
        channel = self.channel_input.value()
        calibration = next(
            (
                item
                for item in load_calibrations()
                if (item["board"], item["channel"]) == (board, channel)
            ),
            None,
        )
        if calibration is None:
            self.existing_label.setText(
                "No saved calibration for this sensor."
            )
        else:
            self.existing_label.setText(
                "Saved calibration: "
                f"scale={calibration['scale']:.9g}, "
                f"bias={calibration['bias']:.9g}"
            )

    def clear_config(self):
        answer = QtWidgets.QMessageBox.question(
            self,
            "Clear storage calibration",
            f"Remove the entire calibration file?\n{CALIB_PATH}",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return

        if os.path.exists(CALIB_PATH):
            os.remove(CALIB_PATH)
        self.ros_node.clear_driver_calibration()
        self.result_label.setText("Scale: ---\nBias: ---")
        self.update_existing_label()


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    ros_node = StorageCalibNode(measurement_callback=lambda *args: None)
    gui = StorageCalibApp(ros_node)

    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(
        lambda: rclpy.spin_once(ros_node, timeout_sec=0.0)
    )
    ros_timer.start(10)
    gui.show()
    exit_code = app.exec_()
    ros_timer.stop()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
