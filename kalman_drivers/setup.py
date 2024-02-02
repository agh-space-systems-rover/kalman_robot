from setuptools import setup, find_packages
import glob
import os
import xml.etree.ElementTree as ET
import sys
import subprocess

# If compasscal is not built, build it.
if not os.path.exists(os.path.join("compasscal_build", "compasscal")):
    # Configure compasscal.
    src_dir = os.path.abspath(sys.path[0])
    compasscal_src_dir = os.path.abspath(os.path.join(src_dir, "compasscal_src"))
    compasscal_build_dir = os.path.abspath(os.path.join(src_dir, "compasscal_build"))
    os.makedirs(compasscal_build_dir, exist_ok=True)
    distro = os.environ["ROS_DISTRO"]

    # Run aclocal, autoconf, automake
    result = subprocess.run(
        f"aclocal && autoconf && autoheader && automake --add-missing",
        cwd=compasscal_src_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True,
    )

    # Check if command has failed.
    if result.returncode != 0:
        output = result.stdout.decode("utf-8")
        print("Compasscal autotools failed:\n" + output, file=sys.stderr)
        sys.exit(1)

    result = subprocess.run(
        f'CFLAGS="-I/opt/ros/humble/opt/libphidget22/include/libphidget22 -I{compasscal_src_dir}/compasscal_lib" LDFLAGS="-L/opt/ros/humble/opt/libphidget22/lib" {compasscal_src_dir}/configure',
        cwd=compasscal_build_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        shell=True,
    )

    # Check if configure failed.
    if not os.path.exists(os.path.join(compasscal_build_dir, "Makefile")):
        # If configure failed, log the error and exit.
        output = result.stdout.decode("utf-8")
        print("Compasscal configure failed:\n" + output, file=sys.stderr)
        sys.exit(1)

    # Configure succeeded. Build compasscal.
    result = subprocess.run(
        ["make"],
        cwd=compasscal_build_dir,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )

    # Check if make failed.
    if not os.path.exists(os.path.join(compasscal_build_dir, "compasscal")):
        # If make failed, log the error and exit.
        output = result.stdout.decode("utf-8")
        print("Compasscal make failed:\n" + output, file=sys.stderr)
        sys.exit(1)

# Parse package.xml to extract package information.
tree = ET.parse("package.xml")
root = tree.getroot()
package_name = root.find("name").text
package_version = root.find("version").text
maintainer_name = root.find("maintainer").text
maintainer_email = root.find("maintainer").get("email")
description = root.find("description").text
license = root.find("license").text

setup(
    name=package_name,
    version=package_version,
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "compasscal_build/compasscal"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            "share/" + package_name + "/param",
            glob.glob(os.path.join("param", "*")),
        ),
    ],
    zip_safe=True,
    maintainer=maintainer_name,
    maintainer_email=maintainer_email,
    description=description,
    license=license,
    entry_points={
        "console_scripts": [
            "compasscal = drivers.compasscal_node:main",
        ],
    },
)
