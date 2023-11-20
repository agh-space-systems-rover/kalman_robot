import os
from datetime import date, datetime
from ros2pkg.api import get_prefix_path

from geometry_msgs.msg import Vector3

class GpsLogger:
    """
    Logs gps data to a file.

    Args:
        filename_prefix (str): The prefix of the filename. The date will be appended to the filename.

    Attributes:
        path (str): The path to the log file.
        filename_prefix (str): The prefix of the filename. The date will be appended to the filename.

    Usage:
        logger = GpsLogger('gps')
        logger.log(Vector3(1, 2, 3), 'This is a description')
    """
    def __init__(self, filename_prefix: str) -> None:
        today = date.today()
        dir_path = os.path.join(get_prefix_path("kalman_groundstation"), 'log')
        self.path = os.path.join(dir_path, f'{filename_prefix}_{today}') 
        self.filename_prefix = filename_prefix

    def log(self, gps: Vector3, description: str = ''):
        with open(self.path, 'a') as f:
            line = f'{datetime.now().strftime("%H:%M:%S")} {gps.x:.6f}, {gps.y:.6f}, {gps.z:.6f} {description}\n'
            f.write(line)