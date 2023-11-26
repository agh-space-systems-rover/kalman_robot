from setuptools import setup, find_packages
import glob
import os
import xml.etree.ElementTree as ET
import sys

# This is a 🕵️ hack 🕵️ that allows us to find the root of the source directory during runtime.
# Save current source path to a temporary data file.
src_dir = os.path.abspath(sys.path[0])
src_dir_filename = "src_dir"
src_dir_file = os.path.join(src_dir, src_dir_filename)
with open(src_dir_file, "w") as f:
    # Exploit the fact that the current directory is the root of the source directory.
    f.write(src_dir)

# Parse package.xml to extract package information.
tree = ET.parse("package.xml")
root = tree.getroot()
package_name = root.find("name").text
package_version = root.find("version").text
maintainer_name = root.find("maintainer").text
maintainer_email = root.find("maintainer").get("email")
description = root.find("description").text
license = root.find("license").text

try:
    setup(
        name=package_name,
        version=package_version,
        packages=find_packages(),
        data_files=[
            ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
            ("share/" + package_name, ["package.xml"]),
            (
                os.path.join("share", package_name, "launch"),
                glob.glob(os.path.join("launch", "*launch.[pxy][yma]*")),
            ),
            (
                "share/" + package_name + "/param",
                glob.glob(os.path.join("param", "*")),
            ),
            (
                "share/" + package_name,
                ["package.xml", src_dir_filename],
            ),  # Install the hack.
        ],
        zip_safe=True,
        maintainer=maintainer_name,
        maintainer_email=maintainer_email,
        description=description,
        license=license,
        entry_points={
            "console_scripts": [
                "wheel_driver = drivers.wheel_driver:main",
            ],
        },
    )
except Exception as e:
    # Remove temporary data file.
    os.remove(src_dir_file)
    raise e
os.remove(src_dir_file)
