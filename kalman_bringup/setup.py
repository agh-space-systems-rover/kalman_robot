from setuptools import setup
import glob
import os
import xml.etree.ElementTree as ET

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
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            "share/" + package_name + "/rviz",
            glob.glob(os.path.join("rviz", "*")),
        ),
    ],
    zip_safe=True,
    maintainer=maintainer_name,
    maintainer_email=maintainer_email,
    description=description,
    license=license,
    tests_require=['pytest'],
)
