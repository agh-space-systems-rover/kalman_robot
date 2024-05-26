from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    arm_controller = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('kalman_arm_config'), 'launch'),
         '/arm_controller.launch.py'])
      )
    
    master = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('kalman_arm_config'), 'launch'),
         '/master.launch.py'])
      )
    
    servo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('kalman_arm_config'), 'launch'),
         '/servo.launch.py'])
      )
    
    drivers = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('kalman_arm_config'), 'launch'),
         '/drivers.launch.py'])
      )
    
    return LaunchDescription([
        arm_controller,
        master,
        servo,
        drivers
    ])