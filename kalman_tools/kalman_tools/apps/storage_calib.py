import os
import struct
import sys
import threading

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
        if len(message.data) < 6:
            return
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
        self.waiting_for_measurement = [False, False]
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
        for number in range(1, 3):
            group = QtWidgets.QGroupBox(f"Measurement {number}")
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
                lambda _, index=number - 1: self.request_measurement(index)
            )
            form.addRow("", measure_button)
            calibration_layout.addWidget(group)

            self.known_inputs.append(known_input)
            self.measured_inputs.append(measured_input)
            self.measure_buttons.append(measure_button)

        self.result_label = QtWidgets.QLabel("Scale: ---\nBias: ---")
        calibration_layout.addWidget(self.result_label)

        self.measurement_status = QtWidgets.QLabel()
        calibration_layout.addWidget(self.measurement_status)

        self.measurement_loader = QtWidgets.QProgressBar()
        self.measurement_loader.setRange(0, 0)
        self.measurement_loader.setTextVisible(False)
        self.measurement_loader.hide()
        calibration_layout.addWidget(self.measurement_loader)

        self.save_button = QtWidgets.QPushButton("Calculate and save")
        self.save_button.clicked.connect(self.calculate_and_save)
        calibration_layout.addWidget(self.save_button)

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
            "Remove selected board + channel"
        )
        self.remove_button.clicked.connect(self.remove_selected_calibration)
        config_layout.addWidget(self.remove_button)

        self.board_input.valueChanged.connect(self.update_existing_label)
        self.channel_input.valueChanged.connect(self.update_existing_label)
        self.update_existing_label()
        self.refresh_config_table()
        self.resize(900, 720)

    @staticmethod
    def make_value_input():
        value_input = CompactDoubleSpinBox()
        value_input.setRange(-1_000_000_000.0, 1_000_000_000.0)
        value_input.setDecimals(6)
        return value_input

    def calculate_and_save(self):
        measured_values = [
            measured_input.value()
            for measured_input in self.measured_inputs
        ]

        try:
            scale, bias = calculate_calibration(
                self.known_inputs[0].value(),
                measured_values[0],
                self.known_inputs[1].value(),
                measured_values[1],
            )
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, "Cannot calibrate", str(error))
            return

        board = self.board_input.value()
        channel = self.channel_input.value()
        save_calibration(board, channel, scale, bias)
        self.result_label.setText(f"Scale: {scale:.9g}\nBias: {bias:.9g}")
        self.plot_calibration(measured_values, scale, bias)
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
        self.waiting_for_measurement[index] = True
        self.measurement_status.setText(
            f"Waiting for board {board}, channel {channel}..."
        )
        self.measurement_loader.show()
        self.ros_node.request_measurement(board, channel)

    def handle_measurement(self, board, channel, value):
        if self.waiting_for_measurement[0]:
            measurement_index = 0
        elif self.waiting_for_measurement[1]:
            measurement_index = 1
        else:
            return

        self.measured_inputs[measurement_index].setValue(value)
        self.waiting_for_measurement[measurement_index] = False
        self.measurement_status.setText(
            f"Received raw value {value} from board {board}, "
            f"channel {channel}."
        )
        if not any(self.waiting_for_measurement):
            self.measurement_loader.hide()

    def plot_calibration(self, measured_values, scale, bias):
        raw_min, raw_max = sorted(measured_values)
        raw_span = raw_max - raw_min
        padding = raw_span * 0.1 or 1.0
        line_x = [raw_min - padding, raw_max + padding]
        line_y = [scale * raw + bias for raw in line_x]

        self.figure.clear()
        axes = self.figure.add_subplot(111)
        axes.scatter(
            measured_values,
            [known_input.value() for known_input in self.known_inputs],
            label="Measurements",
            zorder=2,
        )
        axes.plot(line_x, line_y, color="red", label="Calibration")
        axes.set_xlabel("Measured/raw value")
        axes.set_ylabel("Known weight [g]")
        axes.grid(True, alpha=0.3)
        axes.legend()
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
