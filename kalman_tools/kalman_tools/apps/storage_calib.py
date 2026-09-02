import os
import struct
import sys
import threading

import numpy as np
import rclpy
import yaml
from PyQt5 import QtCore, QtWidgets
from kalman_interfaces.msg import MasterMessage
from matplotlib.backends.backend_qt5agg import (
    FigureCanvasQTAgg as FigureCanvas,
)
from matplotlib.figure import Figure
from rclpy.node import Node


CALIB_PATH = os.path.expanduser("~/.config/kalman/storage_calib.yaml")


class MeasurementBridge(QtCore.QObject):
    received = QtCore.pyqtSignal(int, int, int)


class CompactDoubleSpinBox(QtWidgets.QDoubleSpinBox):
    def textFromValue(self, value):
        text = f"{value:.{self.decimals()}f}".rstrip("0").rstrip(".")
        return text if "." in text else f"{text}.0"


def calculate_calibration(known_values, measured_values):
    scale, bias, _, _ = calculate_robust_calibration(
        known_values,
        measured_values,
    )
    return scale, bias


def calculate_robust_calibration(known_values, measured_values):
    known = np.asarray(known_values, dtype=float)
    measured = np.asarray(measured_values, dtype=float)
    if len(known) < 2:
        raise ValueError("At least two measurements are required.")
    if len(np.unique(measured)) < 2:
        raise ValueError("Measured/raw values must be different.")

    inliers = np.ones(len(known), dtype=bool)
    if len(known) >= 4:
        tolerance = max(float(np.ptp(known)) * 0.02, 1e-9)
        best_score = None
        best_inliers = inliers

        for first in range(len(known) - 1):
            for second in range(first + 1, len(known)):
                raw_delta = measured[second] - measured[first]
                if raw_delta == 0:
                    continue

                candidate_scale = (
                    (known[second] - known[first]) / raw_delta
                )
                candidate_bias = (
                    known[first] - candidate_scale * measured[first]
                )
                residuals = np.abs(
                    known - (candidate_scale * measured + candidate_bias)
                )
                candidate_inliers = residuals <= tolerance
                inlier_count = int(np.count_nonzero(candidate_inliers))
                if inlier_count < 2:
                    continue

                inlier_error = float(
                    np.mean(residuals[candidate_inliers] ** 2)
                )
                score = (inlier_count, -inlier_error)
                if best_score is None or score > best_score:
                    best_score = score
                    best_inliers = candidate_inliers

        inliers = best_inliers

    scale, bias = np.polyfit(measured[inliers], known[inliers], 1)
    predicted = scale * measured[inliers] + bias
    residual_sum = float(np.sum((known[inliers] - predicted) ** 2))
    total_sum = float(
        np.sum((known[inliers] - np.mean(known[inliers])) ** 2)
    )
    r_squared = 1.0 if total_sum == 0 else 1.0 - residual_sum / total_sum
    return float(scale), float(bias), inliers, r_squared


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


def remove_calibration(board, channel):
    calibrations = [
        calibration
        for calibration in load_calibrations()
        if (calibration["board"], calibration["channel"]) != (board, channel)
    ]

    if calibrations:
        with open(CALIB_PATH, "w") as file:
            yaml.safe_dump(calibrations, file, sort_keys=False)
    elif os.path.exists(CALIB_PATH):
        os.remove(CALIB_PATH)


def clear_calibrations():
    if os.path.exists(CALIB_PATH):
        os.remove(CALIB_PATH)


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

    def request_measurement(self, board, channel):
        request = MasterMessage()
        request.cmd = MasterMessage.SCALE_REQ
        request.data = struct.pack("BB", board, channel)
        self.request_pub.publish(request)

    def handle_response(self, message):
        board, channel, value = struct.unpack("<BBi", bytes(message.data[:6]))
        self.measurement_callback(board, channel, value)


class StorageCalibApp(QtWidgets.QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.measurement_bridge = MeasurementBridge(self)
        self.measurement_bridge.received.connect(self.handle_measurement)
        self.ros_node.measurement_callback = (
            self.measurement_bridge.received.emit
        )
        self.waiting_for_measurement = []
        self.expected_sensors = []
        self.calculated_calibration = None
        self.setWindowTitle("Storage Calibration")

        layout = QtWidgets.QHBoxLayout(self)
        calibration_layout = QtWidgets.QVBoxLayout()
        config_layout = QtWidgets.QVBoxLayout()
        layout.addLayout(calibration_layout, 3)
        layout.addLayout(config_layout, 2)

        sensor_group = QtWidgets.QGroupBox("Sensor")
        sensor_layout = QtWidgets.QFormLayout(sensor_group)
        self.board_input = QtWidgets.QSpinBox()
        self.board_input.setRange(0, 255)
        self.channel_input = QtWidgets.QSpinBox()
        self.channel_input.setRange(0, 255)
        sensor_layout.addRow("Board:", self.board_input)
        sensor_layout.addRow("Channel:", self.channel_input)
        calibration_layout.addWidget(sensor_group)

        self.known_inputs = []
        self.measured_inputs = []
        self.measure_buttons = []
        measurements_container = QtWidgets.QWidget()
        self.measurements_layout = QtWidgets.QVBoxLayout(
            measurements_container
        )
        self.measurements_layout.setContentsMargins(0, 0, 0, 0)
        measurements_scroll = QtWidgets.QScrollArea()
        measurements_scroll.setWidgetResizable(True)
        measurements_scroll.setWidget(measurements_container)
        measurements_scroll.setMinimumHeight(260)
        calibration_layout.addWidget(measurements_scroll)

        self.add_measurement()
        self.add_measurement()

        self.add_measurement_button = QtWidgets.QPushButton(
            "+ Add measurement"
        )
        self.add_measurement_button.clicked.connect(self.add_measurement)
        calibration_layout.addWidget(self.add_measurement_button)

        self.result_label = QtWidgets.QLabel("Scale: ---\nBias: ---")
        calibration_layout.addWidget(self.result_label)

        self.measurement_status = QtWidgets.QLabel()
        calibration_layout.addWidget(self.measurement_status)

        self.measurement_loader = QtWidgets.QProgressBar()
        self.measurement_loader.setRange(0, 0)
        self.measurement_loader.setTextVisible(False)
        self.measurement_loader.hide()
        calibration_layout.addWidget(self.measurement_loader)

        calibration_actions = QtWidgets.QHBoxLayout()
        self.calculate_button = QtWidgets.QPushButton("Calculate")
        self.calculate_button.clicked.connect(self.calculate)
        calibration_actions.addWidget(self.calculate_button)
        self.save_button = QtWidgets.QPushButton("Save")
        self.save_button.clicked.connect(self.save_result)
        self.save_button.setEnabled(False)
        calibration_actions.addWidget(self.save_button)
        calibration_layout.addLayout(calibration_actions)

        self.existing_label = QtWidgets.QLabel()
        calibration_layout.addWidget(self.existing_label)

        self.figure = Figure(figsize=(4.2, 3.0))
        self.canvas = FigureCanvas(self.figure)
        calibration_layout.addWidget(self.canvas)

        self.clear_button = QtWidgets.QPushButton("Clear entire config")
        self.clear_button.clicked.connect(self.clear_config)
        calibration_layout.addWidget(self.clear_button)

        config_layout.addWidget(QtWidgets.QLabel("Current config"))
        self.config_table = QtWidgets.QTableWidget(0, 4)
        self.config_table.setHorizontalHeaderLabels(
            ["Board", "Channel", "Scale", "Bias"]
        )
        self.config_table.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows
        )
        self.config_table.setSelectionMode(
            QtWidgets.QAbstractItemView.SingleSelection
        )
        self.config_table.setEditTriggers(
            QtWidgets.QAbstractItemView.NoEditTriggers
        )
        self.config_table.horizontalHeader().setSectionResizeMode(
            QtWidgets.QHeaderView.ResizeToContents
        )
        self.config_table.horizontalHeader().setStretchLastSection(True)
        config_layout.addWidget(self.config_table)

        self.remove_button = QtWidgets.QPushButton(
            "Remove selected board and channel"
        )
        self.remove_button.clicked.connect(self.remove_selected_calibration)
        config_layout.addWidget(self.remove_button)

        self.board_input.valueChanged.connect(self.update_existing_label)
        self.channel_input.valueChanged.connect(self.update_existing_label)
        self.update_existing_label()
        self.refresh_config_table()
        self.resize(900, 720)

    def add_measurement(self):
        index = len(self.known_inputs)
        group = QtWidgets.QGroupBox(f"Measurement {index + 1}")
        form = QtWidgets.QFormLayout(group)

        known_input = self.make_value_input()
        known_input.setSuffix(" g")
        measured_input = self.make_value_input()
        measured_input.setSpecialValueText("---")
        measured_input.setValue(measured_input.minimum())
        measured_input.setEnabled(False)
        form.addRow("Known weight:", known_input)
        form.addRow("Measured/raw value:", measured_input)

        measure_button = QtWidgets.QPushButton("Measure")
        measure_button.clicked.connect(
            lambda _, measurement_index=index: self.request_measurement(
                measurement_index
            )
        )
        form.addRow("", measure_button)
        self.measurements_layout.addWidget(group)

        self.known_inputs.append(known_input)
        self.measured_inputs.append(measured_input)
        self.measure_buttons.append(measure_button)
        self.waiting_for_measurement.append(False)
        self.expected_sensors.append(None)
        known_input.valueChanged.connect(self.invalidate_calculation)
        measured_input.valueChanged.connect(self.invalidate_calculation)

    @staticmethod
    def make_value_input():
        value_input = CompactDoubleSpinBox()
        value_input.setRange(-1_000_000_000.0, 1_000_000_000.0)
        value_input.setDecimals(6)
        return value_input

    def invalidate_calculation(self):
        self.calculated_calibration = None
        if hasattr(self, "save_button"):
            self.save_button.setEnabled(False)

    def calculate(self):
        points = [
            (measured_input.value(), known_input.value())
            for known_input, measured_input in zip(
                self.known_inputs, self.measured_inputs
            )
            if measured_input.text() != "---"
        ]
        if len(points) < 2:
            QtWidgets.QMessageBox.warning(
                self,
                "Cannot calibrate",
                "At least two measurements are required.",
            )
            return

        measured_values, known_values = map(list, zip(*points))

        try:
            scale, bias, inliers, r_squared = calculate_robust_calibration(
                known_values,
                measured_values,
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            QtWidgets.QMessageBox.warning(self, "Cannot calibrate", str(error))
            return

        self.calculated_calibration = (scale, bias)
        inlier_count = int(np.count_nonzero(inliers))
        self.result_label.setText(
            f"Scale: {scale:.9g}\n"
            f"Bias: {bias:.9g}\n"
            f"R²: {r_squared:.6f}\n"
            f"Used points: {inlier_count}/{len(inliers)}"
        )
        self.plot_calibration(
            measured_values,
            known_values,
            inliers,
            scale,
            bias,
        )
        self.save_button.setEnabled(True)

    def save_result(self):
        if self.calculated_calibration is None:
            return

        scale, bias = self.calculated_calibration
        board = self.board_input.value()
        channel = self.channel_input.value()
        save_calibration(board, channel, scale, bias)
        self.update_existing_label()
        self.refresh_config_table()
        QtWidgets.QMessageBox.information(
            self,
            "Calibration saved",
            f"Saved board {board}, channel {channel} to:\n{CALIB_PATH}",
        )

    def request_measurement(self, index):
        board = self.board_input.value()
        channel = self.channel_input.value()
        self.invalidate_calculation()
        self.waiting_for_measurement[index] = True
        self.expected_sensors[index] = (board, channel)
        self.measurement_status.setText(
            f"Waiting for board {board}, channel {channel}..."
        )
        self.measurement_loader.show()
        self.ros_node.request_measurement(board, channel)

    def handle_measurement(self, board, channel, value):
        sensor = (board, channel)
        measurement_index = next(
            (
                index
                for index, waiting in enumerate(
                    self.waiting_for_measurement
                )
                if waiting and self.expected_sensors[index] == sensor
            ),
            None,
        )
        if measurement_index is None:
            return

        self.measured_inputs[measurement_index].setValue(value)
        self.waiting_for_measurement[measurement_index] = False
        self.expected_sensors[measurement_index] = None
        self.measurement_status.setText(
            f"Received raw value {value} from board {board}, "
            f"channel {channel}."
        )
        if not any(self.waiting_for_measurement):
            self.measurement_loader.hide()

    def plot_calibration(
        self,
        measured_values,
        known_values,
        inliers,
        scale,
        bias,
    ):
        measured_values = np.asarray(measured_values)
        known_values = np.asarray(known_values)
        raw_min = min(measured_values)
        raw_max = max(measured_values)
        raw_span = raw_max - raw_min
        padding = raw_span * 0.1 or 1.0
        line_x = [raw_min - padding, raw_max + padding]
        line_y = [scale * raw + bias for raw in line_x]

        self.figure.clear()
        axes = self.figure.add_subplot(111)
        axes.scatter(
            measured_values[inliers],
            known_values[inliers],
            zorder=2,
        )
        if not np.all(inliers):
            axes.scatter(
                measured_values[~inliers],
                known_values[~inliers],
                color="orange",
                marker="x",
                s=60,
                zorder=3,
            )
        axes.plot(line_x, line_y, color="red")
        axes.set_xlabel("Raw")
        axes.set_ylabel("Weight [g]")
        axes.grid(True, alpha=0.3)
        self.figure.tight_layout()
        self.canvas.draw()

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

    def refresh_config_table(self):
        calibrations = load_calibrations()
        self.config_table.setRowCount(len(calibrations))
        for row, calibration in enumerate(calibrations):
            values = (
                str(calibration["board"]),
                str(calibration["channel"]),
                f"{calibration['scale']:.9g}",
                f"{calibration['bias']:.9g}",
            )
            for column, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value)
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.config_table.setItem(row, column, item)

        self.remove_button.setEnabled(bool(calibrations))

    def remove_selected_calibration(self):
        row = self.config_table.currentRow()
        if row < 0:
            QtWidgets.QMessageBox.warning(
                self,
                "No calibration selected",
                "Select a board + channel row to remove.",
            )
            return

        board = int(self.config_table.item(row, 0).text())
        channel = int(self.config_table.item(row, 1).text())
        answer = QtWidgets.QMessageBox.question(
            self,
            "Remove calibration",
            f"Remove calibration for board {board}, channel {channel}?",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return

        remove_calibration(board, channel)
        self.update_existing_label()
        self.refresh_config_table()

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

        clear_calibrations()
        self.invalidate_calculation()
        self.result_label.setText("Scale: ---\nBias: ---")
        self.update_existing_label()
        self.refresh_config_table()


def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    ros_node = StorageCalibNode(measurement_callback=lambda *args: None)
    gui = StorageCalibApp(ros_node)

    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(ros_node,),
        daemon=True,
    )
    ros_thread.start()
    gui.show()
    exit_code = app.exec_()
    rclpy.shutdown()
    ros_thread.join(timeout=2.0)
    ros_node.destroy_node()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
