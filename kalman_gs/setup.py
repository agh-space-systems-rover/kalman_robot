from setuptools import setup

package_name = 'kalman_gs'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
      install_requires=['setuptools', 'rclpy', 'std_srvs'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    maintainer='Your Maintainer Name',
    maintainer_email='maintainer.email@example.com',
    description='A description of your package',
    license='Your License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = kalman_gs.restart_joints_client:main',
            'client = kalman_gs.estart_joints_service:main',
        ],
    },
)
