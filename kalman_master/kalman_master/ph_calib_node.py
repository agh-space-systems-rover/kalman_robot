import sys
import struct
import os
import yaml
import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from PyQt5 import QtWidgets, QtCore
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

CALIB_PATH = os.path.expanduser("~/.config/kalman/ph_calib.yaml")

class PHCalibNode(Node):
    def __init__(self, gui_callback):
        super().__init__("ph_calib_gui")
        self.value_raw_sub = self.create_subscription(
            Float32, "science/ph/value/raw", self.cb_value_raw, 10
        )
        # Replace publisher with service client
        self.value_trigger_cli = self.create_client(Trigger, "science/ph/value/req")
        self.latest_sem = None
        self.last_update_time = None
        self.gui_callback = gui_callback

    def cb_value_raw(self, msg):
        self.latest_sem = msg.data
        self.last_update_time = time.time()
        self.gui_callback(msg.data)

    def request_value(self):
        # Wait for service if needed
        if not self.value_trigger_cli.wait_for_service(timeout_sec=1.0):
            return
        req = Trigger.Request()
        future = self.value_trigger_cli.call_async(req)
        # Optionally handle future result if needed

class PHCalibApp(QtWidgets.QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.ros_node.gui_callback = self.on_sem_update
        self.sem_value = None
        self.last_update_time = None
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_timer)
        self.timer.start(100)
        self.datapoints = []
        self.init_ui()
        self.load_and_plot_calib()  # <-- Load and plot calibration on startup

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        # SEM value and timer
        self.sem_label = QtWidgets.QLabel("SEM: ---")
        self.ph_label = QtWidgets.QLabel("pH: ---")
        sem_ph_layout = QtWidgets.QHBoxLayout()
        sem_ph_layout.addWidget(self.sem_label)
        sem_ph_layout.addWidget(self.ph_label)
        layout.addLayout(sem_ph_layout)
        self.timer_label = QtWidgets.QLabel("Last update: --- s ago")
        layout.addWidget(self.timer_label)

        # Request button
        self.req_btn = QtWidgets.QPushButton("Request SEM value")
        self.req_btn.clicked.connect(self.request_value)
        layout.addWidget(self.req_btn)

        # pH buttons
        ph_btn_layout = QtWidgets.QHBoxLayout()
        self.ph_buttons = []
        for i in range(15):
            btn = QtWidgets.QPushButton(str(i))
            btn.clicked.connect(lambda _, ph=i: self.add_datapoint(ph))
            self.ph_buttons.append(btn)
            ph_btn_layout.addWidget(btn)
        layout.addLayout(ph_btn_layout)

        # Datapoint table with header
        self.datapoint_table = QtWidgets.QTableWidget(0, 2)
        self.datapoint_table.setHorizontalHeaderLabels(["SEM", "pH"])
        self.datapoint_table.horizontalHeader().setStretchLastSection(True)
        self.datapoint_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.datapoint_table.horizontalHeader().setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        self.datapoint_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.datapoint_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        layout.addWidget(self.datapoint_table)

        # Remove button
        self.remove_btn = QtWidgets.QPushButton("Remove selected datapoint")
        self.remove_btn.clicked.connect(self.remove_datapoint)
        layout.addWidget(self.remove_btn)

        # Remove all button
        self.remove_all_btn = QtWidgets.QPushButton("Remove all datapoints")
        self.remove_all_btn.clicked.connect(self.remove_all_datapoints)
        layout.addWidget(self.remove_all_btn)

        # Calibrate button
        self.calib_btn = QtWidgets.QPushButton("Calibrate")
        self.calib_btn.clicked.connect(self.calibrate)
        layout.addWidget(self.calib_btn)

        # Matplotlib plot
        self.figure = Figure(figsize=(2.5, 3))  # Decreased width twofold
        self.canvas = FigureCanvas(self.figure)
        layout.addWidget(self.canvas)

        self.setWindowTitle("pH Calibration GUI")
        self.setLayout(layout)
        self.setFixedWidth(500)  # Set window width to a smaller value

    def on_sem_update(self, sem):
        self.sem_value = sem
        self.last_update_time = time.time()
        self.sem_label.setText(f"SEM: {sem:.3f}")
        ph_val = "---"
        scale = None
        bias = None
        datapoints = self.datapoints
        if os.path.exists(CALIB_PATH):
            try:
                with open(CALIB_PATH, "r") as f:
                    calib_data = yaml.safe_load(f)
                if "scale" in calib_data and "bias" in calib_data:
                    scale = calib_data["scale"]
                    bias = calib_data["bias"]
                    ph_val = scale * sem + bias
                    self.ph_label.setText(f"pH: {ph_val:.3f}")
                else:
                    self.ph_label.setText("pH: ---")
                # Use latest datapoints from file for plotting
                if "data" in calib_data and isinstance(calib_data["data"], list):
                    datapoints = [
                        (dp["sem"], dp["ph"])
                        for dp in calib_data["data"]
                        if "sem" in dp and "ph" in dp
                    ]
            except Exception:
                self.ph_label.setText("pH: ---")
        else:
            self.ph_label.setText("pH: ---")

        # Update plot with vertical/horizontal lines
        if scale is not None and bias is not None and len(datapoints) >= 2:
            sems, phs = zip(*datapoints)
            sems = np.array(sems)
            phs = np.array(phs)
            fit_phs = scale * sems + bias
            self.figure.clear()
            ax = self.figure.add_subplot(111)
            ax.scatter(sems, phs, label="Datapoints")
            ax.plot(sems, fit_phs, color="red", label="Fit")
            ax.set_xlabel("SEM")
            ax.set_ylabel("pH")
            ax.legend()
            # Add vertical line at measured SEM (now green)
            ax.axvline(sem, color="green", linestyle="-", linewidth=1)
            # Add horizontal dashed line at remapped pH
            if ph_val != "---":
                ax.axhline(ph_val, color="gray", linestyle="--", linewidth=1)
            self.canvas.draw()

    def update_timer(self):
        if self.last_update_time:
            elapsed = time.time() - self.last_update_time
            self.timer_label.setText(f"Last update: {elapsed:.1f} s ago")
        else:
            self.timer_label.setText("Last update: --- s ago")

    def request_value(self):
        self.ros_node.request_value()

    def add_datapoint(self, ph):
        if self.sem_value is None:
            QtWidgets.QMessageBox.warning(self, "Error", "No SEM value received yet.")
            return
        self.datapoints.append((self.sem_value, ph))
        self.update_datapoint_list()

    def update_datapoint_list(self):
        self.datapoint_table.setRowCount(len(self.datapoints))
        for row, (sem, ph) in enumerate(self.datapoints):
            sem_item = QtWidgets.QTableWidgetItem(f"{sem:.3f}")
            ph_item = QtWidgets.QTableWidgetItem(f"{ph}")
            sem_item.setTextAlignment(QtCore.Qt.AlignCenter)
            ph_item.setTextAlignment(QtCore.Qt.AlignCenter)
            self.datapoint_table.setItem(row, 0, sem_item)
            self.datapoint_table.setItem(row, 1, ph_item)

    def remove_datapoint(self):
        idx = self.datapoint_table.currentRow()
        if idx >= 0:
            self.datapoints.pop(idx)
            self.update_datapoint_list()

    def remove_all_datapoints(self):
        self.datapoints.clear()
        self.update_datapoint_list()

    def calibrate(self):
        if len(self.datapoints) < 2:
            QtWidgets.QMessageBox.warning(self, "Error", "Need at least 2 datapoints.")
            return
        sems, phs = zip(*self.datapoints)
        sems = np.array(sems)
        phs = np.array(phs)
        A = np.vstack([sems, np.ones(len(sems))]).T
        scale, bias = np.linalg.lstsq(A, phs, rcond=None)[0]
        # Save calibration and datapoints
        os.makedirs(os.path.dirname(CALIB_PATH), exist_ok=True)
        calib_dict = {
            "scale": float(scale),
            "bias": float(bias),
            "data": [{"sem": float(sem), "ph": float(ph)} for sem, ph in self.datapoints]
        }
        with open(CALIB_PATH, "w") as f:
            yaml.safe_dump(calib_dict, f)
        # Plot
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.scatter(sems, phs, label="Datapoints")
        fit_phs = scale * sems + bias
        ax.plot(sems, fit_phs, color="red", label="Fit")
        ax.set_xlabel("SEM")
        ax.set_ylabel("pH")
        ax.legend()
        self.canvas.draw()
        QtWidgets.QMessageBox.information(self, "Calibration", f"Calibration saved!\nScale: {scale:.4f}\nBias: {bias:.4f}")

    def load_and_plot_calib(self):
        if os.path.exists(CALIB_PATH):
            try:
                with open(CALIB_PATH, "r") as f:
                    calib_data = yaml.safe_load(f)
                # Load datapoints if available
                if "data" in calib_data and isinstance(calib_data["data"], list):
                    self.datapoints = [
                        (dp["sem"], dp["ph"])
                        for dp in calib_data["data"]
                        if "sem" in dp and "ph" in dp
                    ]
                    self.update_datapoint_list()
                # Plot calibration if scale/bias and datapoints are available
                if (
                    "scale" in calib_data
                    and "bias" in calib_data
                    and len(self.datapoints) >= 2
                ):
                    sems, phs = zip(*self.datapoints)
                    sems = np.array(sems)
                    phs = np.array(phs)
                    scale = calib_data["scale"]
                    bias = calib_data["bias"]
                    self.figure.clear()
                    ax = self.figure.add_subplot(111)
                    ax.scatter(sems, phs, label="Datapoints")
                    fit_phs = scale * sems + bias
                    ax.plot(sems, fit_phs, color="red", label="Fit")
                    ax.set_xlabel("SEM")
                    ax.set_ylabel("pH")
                    ax.legend()
                    self.canvas.draw()
            except Exception:
                pass

def main():
    rclpy.init()
    app = QtWidgets.QApplication(sys.argv)
    ros_node = PHCalibNode(gui_callback=lambda sem: None)
    gui = PHCalibApp(ros_node)
    gui.show()

    # Spin ROS2 in a thread
    def ros_spin():
        rclpy.spin(ros_node)
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()

    sys.exit(app.exec_())
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
