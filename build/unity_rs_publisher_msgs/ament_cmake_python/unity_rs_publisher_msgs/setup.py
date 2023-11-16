from setuptools import find_packages
from setuptools import setup

setup(
    name='unity_rs_publisher_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('unity_rs_publisher_msgs', 'unity_rs_publisher_msgs.*')),
)
