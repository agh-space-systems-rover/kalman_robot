from setuptools import find_packages
from setuptools import setup

setup(
    name='kalman_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('kalman_interfaces', 'kalman_interfaces.*')),
)
