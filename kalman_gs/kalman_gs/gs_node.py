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

        # Run npm install if dependencies were not installed yet.

        # Get last installation time or 0 if deps were never installed.
        marker_file = os.path.expanduser("~/.cache/kalman/last_gs_npm_install_time.txt")
        if os.path.isfile(marker_file):
            with open(marker_file, "r") as f:
                last_gs_npm_install_time = float(f.read())
        else:
            last_gs_npm_install_time = 0
        if not os.path.exists(os.path.join(self.node_project_dir, "node_modules")):
            last_gs_npm_install_time = 0

        # Get the latest modification time of package.json
        latest_package_json_modified = os.path.getmtime(
            os.path.join(self.node_project_dir, "package.json")
        )

        # If package.json was modified after most recent npm install, re-run npm install.
        if latest_package_json_modified > last_gs_npm_install_time:
            self.get_logger().info("Running 'npm install':")
            if self.run_command("npm install") == 0:
                os.makedirs(os.path.dirname(marker_file), exist_ok=True)
                with open(marker_file, "w") as f:
                    f.write(str(latest_package_json_modified))
            else:
                self.get_logger().error("Failed to install npm dependencies.")
                raise KeyboardInterrupt

        # Run npm start
        self.get_logger().info("Running 'npm start':")
        self.run_command("npm start")

    def run_command(self, command: str) -> int:
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
                # But if it is not the first time the user started the node,
                # and the browser is already open, we assume
                # that the ground station is already running.

                # Check if this is the first ever run on this machine.
                first_run_marker = os.path.expanduser(
                    "~/.cache/kalman/first_gs_run.txt"
                )
                if not os.path.isfile(first_run_marker):
                    os.makedirs(os.path.dirname(first_run_marker), exist_ok=True)
                    with open(first_run_marker, "w") as f:
                        f.write(
                            "Existence of this file indicates that the ground station was already started once, and the user will know to open the browser manually."
                        )

                    # Open the browser
                    webbrowser.open("http://localhost:3000")
                else:
                    # If this is not the first run, see if the browser is open.
                    browser_is_open = False
                    for proc in sp.Popen(["ps", "aux"], stdout=sp.PIPE).stdout:
                        if "firefox" in str(proc) or "chromium" in str(proc):
                            browser_is_open = True
                            break
                    if not browser_is_open:
                        webbrowser.open("http://localhost:3000")

            if line:
                self.get_logger().info(line.strip())

        # Read the remaining output
        for line in process.stdout.readlines():
            self.get_logger().info(line.strip())

        # Return the exit code
        return process.returncode


def main():
    try:
        rclpy.init()
        node = GS()
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
