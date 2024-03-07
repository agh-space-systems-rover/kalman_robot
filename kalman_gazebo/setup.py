from setuptools import find_packages, setup
import os
import glob

package_name = "kalman_gazebo"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob.glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (
            "share/" + package_name + "/config",
            glob.glob(os.path.join("config", "*")),
        ),
        (
            "share/" + package_name + "/urdf",
            glob.glob(os.path.join("urdf", "*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="stefe",
    maintainer_email="16748784+FIXMBR@users.noreply.github.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=['pytest'],
    entry_points={
        "console_scripts": ["gazebo_wheel_driver = drivers.gazebo_wheel_driver:main"],
    },
)
