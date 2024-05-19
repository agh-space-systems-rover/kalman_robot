from setuptools import find_packages, setup
import os
from glob import glob

package_name = "kalman_arm_config"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    + [
        (os.path.join("lib", "config", os.path.dirname(path)), [path])
        for path in glob("config" + "/**", recursive=True)
        if os.path.isfile(path)
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="todo@todo.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
