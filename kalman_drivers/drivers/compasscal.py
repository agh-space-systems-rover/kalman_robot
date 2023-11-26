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

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import yaml
from ament_index_python.packages import get_package_share_directory
import subprocess
import os

from kalman_interfaces.srv import CalibrateCompass


CMD_VEL_FREQUENCY = 2


class CompasscalNode(Node):
    def __init__(self):
        super().__init__("compasscal")

        # Init cmd_vel rate.
        self.cmd_vel_rate = self.create_rate(CMD_VEL_FREQUENCY)

        # Init cmd_vel publisher.
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Init services.
        self.create_service(CalibrateCompass, "compasscal/calibrate", self.calibrate)

    def calibrate(
        self, request: CalibrateCompass.Request, response: CalibrateCompass.Response
    ):
        # First ensure that compasscal is built.
        # Read the source path.
        with open(get_package_share_directory("kalman_drivers") + "/src_dir") as f:
            src_dir = f.read()
        compasscal_path = src_dir + "/compasscal"

        # Check if compasscal is built.
        if not os.path.exists(os.path.join(compasscal_path, "compasscal")):
            # If not, build it.
            self.get_logger().info("Compasscal is not built. Building it now...")
            self.get_logger().info("Configuring compasscal...")
            subprocess.run(["./configure"], cwd=compasscal_path)
            self.get_logger().info("Building compasscal...")
            subprocess.run(["make"], cwd=compasscal_path)

        # Run compasscal executable
        self.get_logger().info("Running compasscal...")
        process = subprocess.Popen(
            "./compasscal",
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
            universal_newlines=True,
            cwd=compasscal_path,
        )

        # Choose 3-axis calibration
        self.get_logger().info("Choosing 3-axis calibration...")
        process.stdin.write("3\n")
        process.stdin.flush()

        # Start calibration by sending Enter to stdin
        self.get_logger().info("Starting sampling...")
        process.stdin.write("\n")
        process.stdin.flush()

        # Before performing rotations, check if sampling has started.
        # If not, exit with success=False.
        # First read all currently available output from stdout.
        output = process.stdout.read()

        # If the output does not contain "Sampling...", then sampling has not started.
        if "Sampling..." not in output:
            # Kill the process.
            process.kill()
            # Log the output.
            self.get_logger().error("Compasscal calibration failed:\n" + output)
            response.success = False
            return response

        # Now we are sure that sampling has started.
        # Perform rotations by sending cmd_vel messages.
        n = request.duration * CMD_VEL_FREQUENCY
        for i in range(n):
            # Send a cmd_vel message.
            msg = Twist()
            msg.angular.z = request.angular_velocity
            self.cmd_vel_pub.publish(msg)

            # Wait for 1/msg_freq seconds.
            self.cmd_vel_rate.sleep()

        # Stop calibration by sending Enter to stdin
        self.get_logger().info("Stopping sampling...")
        process.stdin.write("\n")
        process.stdin.flush()

        # Read and print final output from stdout
        output = process.communicate()[0]

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

        # If the line was not found, return success=False.
        # Also log the whole output.
        if vars_line is None:
            self.get_logger().error("Compasscal calibration failed:\n" + output)
            response.success = False
            return response

        # Split the line into tokens.
        coeffs = line.split(" ")

        # Drop the first token, "\tVars:"
        coeffs.pop(0)

        # Remove the trailing comma from every other token.
        coeffs = [x.rstrip(",") for x in coeffs]

        # Convert the tokens to floats.
        coeffs = [float(x) for x in coeffs]

        # Print the coefficients.
        self.get_logger().info("Calibration coefficients:\n" + str(coeffs))

        # Save the coefficients to a file.
        with open(
            os.path.join(
                os.path.expanduser("~"),
                ".config/kalman/phidgets_spatial_calibration_params.yaml",
            ),
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


def main():
    rclpy.init()
    node = CompasscalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
