from setuptools import find_packages, setup
import os

package_name = 'wheel_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name), [os.path.join(package_name, package_name + '.py')]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wojciech',
    maintainer_email='wojciech@funsafe.xyz',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_controller = wheel_controller:main'
        ],
    },
)
