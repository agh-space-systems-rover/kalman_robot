import rclpy
from rclpy.node import Node
import subprocess as sp
from ament_index_python.packages import get_package_share_directory
import os
import webbrowser


class GS(Node):
    def __init__(self):
        super().__init__("gs")

        self.get_logger().info("Starting the ground station...")

        self.node_project_dir = (
            get_package_share_directory("kalman_gs") + "/node_project"
        )

        # Run npm install if node_modules does not exist
        if not os.path.exists(self.node_project_dir + "/node_modules"):
            self.get_logger().info("Running 'npm install':")
            self.run_command("npm install")

        # Run npm start
        self.get_logger().info("Running 'npm start':")
        self.run_command("npm start")

    def run_command(self, command):
        # Run and log output line by line
        process = sp.Popen(
            command,
            cwd=self.node_project_dir,
            stdout=sp.PIPE,
            stderr=sp.STDOUT,
            universal_newlines=True,
            shell=True,
        )
        while process.poll() is None:
            line = process.stdout.readline()

            # If line reads "Starting the development server...", open the browser
            if "Starting the development server..." in line:
                webbrowser.open("http://localhost:3000")

            if line:
                self.get_logger().info(line.strip())

        # Read the remaining output
        for line in process.stdout.readlines():
            self.get_logger().info(line.strip())


def main():
    try:
        rclpy.init()
        node = GS()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
