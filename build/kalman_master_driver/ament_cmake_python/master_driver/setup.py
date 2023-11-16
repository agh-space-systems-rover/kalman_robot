from setuptools import find_packages
from setuptools import setup

setup(
    name='master_driver',
    version='0.0.0',
    packages=find_packages(
        include=('master_driver', 'master_driver.*')),
)
