from setuptools import find_packages, setup
import xml.etree.ElementTree as ET
import os
from glob import glob

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
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ],
    zip_safe=True,
    maintainer=maintainer_name,
    maintainer_email=maintainer_email,
    description=description,
    license=license,
    entry_points={
        "console_scripts": [
            "master_com = master_driver.master_com_node:main",
            "ueuos = master_driver.ueuos_node:main"
            "wheel_driver = master_driver.wheel_driver_node:main",
        ],
    },
)
