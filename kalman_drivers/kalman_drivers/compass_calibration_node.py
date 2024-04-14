# Compasscal is a command line tool that calibrates the magnetometer.
# It will ask to choose either a (2) or (3) axis calibration.
# Then the user must input Enter to start the calibration process.
# Then the compass should be rotated in all possible directions.
# Enter is pressed to stop the calibration process.
# 13 calibration coefficients are printed to the terminal -
# - they must be extracted from the human-readable format and
# saved in "~/.config/kalman/compasscal_calibration_params.yaml".

# Below is example output from the program:
# ros@kalman:~$ ./compascal-exe
# Choose:
# 	2: 2-axis calibration
# 	3: 3-axis calibration
#
# ::>3
# Clearing any previous calibration data...
# Press Enter to start sampling...
#
# Sampling... Press Enter to stop...
#
# Estimates: gains(0.000, 0.470, 0.423) offsets(-34593.314, 0.316, -0.463)
# ......
#
# Compass calibration finished
# 	Vars: 3.358053, -34593.313976, 0.315750, -0.462918, 0.000029, 0.470254, 0.423092, -0.000000, 0.000000, -0.000000, -0.000000, 0.000000, -0.000000
#
# These arguments are in the right order to be passed direcly into the PhidgetSpatial_setCompassCorrectionParameters function.
#
# You may wish to use a more accurate value for magnetic field strength as the number provided here is only an estimate.
#
# Calibration values have been written to firmware on your spatial.
#  These values will be maintained from now on, across power cycles, until explicitely reset or changed.
#
# Enter to exit
#
# ros@kalman:~$

import fcntl
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import yaml
from ament_index_python.packages import get_package_prefix
import subprocess
import os

from kalman_interfaces.srv import CalibrateCompass


CMD_VEL_RATE = 2


def async_read_all(stream):
    fd = stream.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    try:
        output = stream.read()
        return output
    except:
        return ""


def format_output(output):
    if len(output.strip()) == 0:
        return "> [NO OUTPUT]"
    return "\n".join(["> " + x for x in output.split("\n")])


def wait_for_process(process, duration):
    try:
        process.wait(duration)
    except:
        pass


class CompasscalNode(Node):
    def __init__(self):
        super().__init__("compass_calibration")

        # Init cmd_vel publisher.
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Init services.
        self.create_service(CalibrateCompass, "calibrate", self.calibrate)

    def calibrate(
        self, request: CalibrateCompass.Request, response: CalibrateCompass.Response
    ):
        # Run compasscal executable
        self.get_logger().info("Starting compasscal.")
        process = subprocess.Popen(
            "stdbuf -o0 "  # Disable buffering. Fixes the problem of not getting any output from compasscal.
            + os.path.join(
                get_package_prefix("kalman_drivers"),
                "lib",
                "kalman_drivers",
                "compasscal",
            ),
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            universal_newlines=True,
            shell=True,
        )

        # Wait until the process has printed the "Choose:" prompt.
        success, output = self.wait_until_output(process, "Choose:")
        if not success:
            self.get_logger().error(
                "compasscal calibration failed:\n" + format_output(output)
            )
            response.success = False
            return response
        else:
            self.get_logger().info(
                "compasscal has successfully started:\n" + format_output(output)
            )

        # Choose 2 or 3-axis calibration
        if request.two_d:
            self.get_logger().info("Choosing 2-axis calibration.")
            process.stdin.write("2\n")
        else:
            self.get_logger().info("Choosing 3-axis calibration.")
            process.stdin.write("3\n")
        process.stdin.flush()

        # Wait until the process has printed the "Press Enter to start sampling..." prompt.
        success, output = self.wait_until_output(
            process, "Press Enter to start sampling..."
        )
        if not success:
            self.get_logger().error(
                "Failed to choose calibration mode:\n" + format_output(output)
            )
            response.success = False
            return response
        else:
            self.get_logger().info(
                "Successfully entered calibration mode:\n" + format_output(output)
            )

        # Start calibration by sending Enter to stdin
        self.get_logger().info("Starting sampling...")
        process.stdin.write("\n")
        process.stdin.flush()

        # Before performing rotations, check if sampling has started.
        # If not, exit with success=False.
        # First read all currently available output from stdout.
        success, output = self.wait_until_output(
            process, "Sampling... Press Enter to stop..."
        )
        if not success:
            self.get_logger().error(
                "Failed to start sampling:\n" + format_output(output)
            )
            response.success = False
            return response
        else:
            self.get_logger().info("Sampling started:\n" + format_output(output))

        # Now we are sure that sampling has started.
        # Perform rotations by sending cmd_vel messages.
        n = int(request.duration * CMD_VEL_RATE)
        for i in range(n):
            self.get_logger().info(f"Rotating the robot: {i+1}/{n}")

            # Send a cmd_vel message.
            msg = Twist()
            # For the second half of the process turn in the reverse direction.
            if i * 2 < n:
                msg.angular.z = request.angular_velocity
            else:
                msg.angular.z = -request.angular_velocity
            self.cmd_vel_pub.publish(msg)

            # Wait for 1/msg_freq seconds.
            wait_for_process(process, 1 / CMD_VEL_RATE)
        self.cmd_vel_pub.publish(Twist())

        # Stop calibration by sending Enter to stdin
        self.get_logger().info("Stopping sampling...")
        process.stdin.write("\n")
        process.stdin.flush()

        # Wait until the process has printed "Enter to exit" prompt.
        success, output = self.wait_until_output(
            process, "explicitely reset or changed.\n\nEnter to exit"
        )
        if not success:
            self.get_logger().error("Calibration failed:\n" + format_output(output))
            response.success = False
            return response
        else:
            self.get_logger().info("Calibration successful:\n" + format_output(output))

        # Exit the program by sending Enter to stdin
        process.stdin.write("\n")
        process.stdin.flush()

        # Wait up to 1 second for the process to exit on its own.
        try:
            process.communicate(timeout=1)[0]
        except:
            # No problem if exit fails.
            pass

        # Find the calibration coefficients in the output
        # They are outputted in this format;
        # Compass calibration finished
        # \tVars: 935.009813, -1514671133.769775, 0.316702, -0.483487, 0.000000, 0.002742, 0.000466, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000

        # Split the output into lines
        lines = output.split("\n")

        # Find the line that starts with "\tVars: "
        vars_line = None
        for line in lines:
            if line.startswith("\tVars: "):
                vars_line = line

        # Split the line into tokens.
        coeffs = vars_line.split(" ")

        # Drop the first token, "\tVars:"
        coeffs.pop(0)

        # Remove the trailing comma from every other token.
        coeffs = [x.rstrip(",") for x in coeffs]

        # Convert the tokens to floats.
        coeffs = [float(x) for x in coeffs]

        # Save the coefficients to a file.
        os.makedirs(os.path.expanduser("~/.config/kalman"), exist_ok=True)
        with open(
            os.path.join(
                os.path.expanduser("~"),
                ".config/kalman/phidgets_spatial_calibration_params.yaml",
            ),
            "w",
        ) as f:
            yaml.dump(
                {
                    "phidgets_spatial": {
                        "ros__parameters": {
                            "cc_mag_field": coeffs[0],
                            "cc_offset0": coeffs[1],
                            "cc_offset1": coeffs[2],
                            "cc_offset2": coeffs[3],
                            "cc_gain0": coeffs[4],
                            "cc_gain1": coeffs[5],
                            "cc_gain2": coeffs[6],
                            "cc_t0": coeffs[7],
                            "cc_t1": coeffs[8],
                            "cc_t2": coeffs[9],
                            "cc_t3": coeffs[10],
                            "cc_t4": coeffs[11],
                            "cc_t5": coeffs[12],
                        }
                    }
                },
                f,
            )

        # Print the coefficients.
        self.get_logger().info("Calibration is done. Coefficients:\n" + str(coeffs))

        # Return success=True.
        response.success = True
        response.cc_mag_field = coeffs[0]
        response.cc_offset0 = coeffs[1]
        response.cc_offset1 = coeffs[2]
        response.cc_offset2 = coeffs[3]
        response.cc_gain0 = coeffs[4]
        response.cc_gain1 = coeffs[5]
        response.cc_gain2 = coeffs[6]
        response.cc_t0 = coeffs[7]
        response.cc_t1 = coeffs[8]
        response.cc_t2 = coeffs[9]
        response.cc_t3 = coeffs[10]
        response.cc_t4 = coeffs[11]
        response.cc_t5 = coeffs[12]
        return response

    def wait_until_output(self, process, target_output):
        output = ""
        for _ in range(50, 0, -1):  # 5 seconds timeout
            output += async_read_all(process.stdout)

            if target_output in output:
                return True, output

            wait_for_process(process, 0.1)

        return False, output


def main():
    try:
        rclpy.init()
        node = CompasscalNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
