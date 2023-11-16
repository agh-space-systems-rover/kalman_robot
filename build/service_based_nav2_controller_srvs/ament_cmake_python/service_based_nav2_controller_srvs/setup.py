from setuptools import find_packages
from setuptools import setup

setup(
    name='service_based_nav2_controller_srvs',
    version='0.0.0',
    packages=find_packages(
        include=('service_based_nav2_controller_srvs', 'service_based_nav2_controller_srvs.*')),
)
