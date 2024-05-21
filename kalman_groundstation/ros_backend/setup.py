from glob import glob
from setuptools import find_packages, setup
import os

package_name = "kalman_groundstation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
    ]
    + [
        (os.path.join('lib', package_name, os.path.dirname(path)), [path]) for
            path in glob(package_name + '/**', recursive=True) if os.path.isfile(path)
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kiwi",
    maintainer_email="kacper.iwi@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "api_node = kalman_groundstation.api_node:main",
            "websocket_node = kalman_groundstation.websocket_node:main",
            "bridge_node = kalman_groundstation.bridge_node:main",
        ],
    },
)
