"""Names of the master_com topics, the drivers' contract with kalman_tunnel.

master_com publishes every frame received from the Master on a topic named
after its frame ID, and sends every MasterMessage published on ROS_TO_MASTER
to the Master. Build the names here instead of by hand in each driver.
"""

# Frames published here are sent to the Master.
ROS_TO_MASTER = "master_com/ros_to_master"


def master_to_ros(cmd: int) -> str:
    """Topic of frames with ID cmd received from the Master, e.g. ".../x5a"."""
    return f"master_com/master_to_ros/x{cmd:x}"
