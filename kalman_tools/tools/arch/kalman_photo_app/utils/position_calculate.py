from utils.trajectory import TrajectoryReader
import numpy as np
from scipy.spatial.transform import Rotation


def calculate_position(
    local_relative_coords, unix_timestamp: int, trajectory: TrajectoryReader
):
    """
    Transform local coordinates relative to the rover into global coordinates.

    Args:
        local_relative_coords: Coordinates in the rover's local frame (x, y, z)
        unix_timestamp: Timestamp used to get the rover's position and orientation
        trajectory: TrajectoryReader object to get rover's pose

    Returns:
        Dictionary with global position {'x': x, 'y': y, 'z': z} or None if rover pose not found
    """
    # Get the rover's position and orientation
    rover_location = trajectory.get_closest_pose_by_timestamp(unix_timestamp)
    print("Rover Location:")
    print(
        f"  Position: x={rover_location['position']['x']}, y={rover_location['position']['y']}, z={rover_location['position']['z']}"
    )
    print(
        f"  Orientation: x={rover_location['orientation']['x']}, y={rover_location['orientation']['y']}, z={rover_location['orientation']['z']}, w={rover_location['orientation']['w']}"
    )
    print("Local Relative Coordinates:")
    print(
        f"  x={local_relative_coords[0]}, y={local_relative_coords[1]}, z={local_relative_coords[2]}"
    )
    if not rover_location:
        return None

    # Convert local coordinates to numpy array
    local_coords = np.array(local_relative_coords)

    # Extract the quaternion components
    quat = [
        rover_location["orientation"]["x"],
        rover_location["orientation"]["y"],
        rover_location["orientation"]["z"],
        rover_location["orientation"]["w"],
    ]

    # Create a rotation object from the quaternion
    r = Rotation.from_quat(quat)

    # Rotate the local coordinates to align with global reference frame
    rotated_coords = r.apply(local_coords)

    # Add the rover's position to get the global coordinates
    rover_position = np.array(
        [
            rover_location["position"]["x"],
            rover_location["position"]["y"],
            rover_location["position"]["z"],
        ]
    )

    global_coords = rotated_coords + rover_position

    print("Global Cube Coordinates:")
    print(f"  x={global_coords[0]}, y={global_coords[1]}, z={global_coords[2]}")
    print("\n")
    print("\n")
    # Return as a dictionary matching the format of rover_location['position']
    return {
        "x": float(global_coords[0]),
        "y": float(global_coords[1]),
        "z": float(global_coords[2]),
    }
