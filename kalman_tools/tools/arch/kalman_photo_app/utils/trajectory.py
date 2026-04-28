import yaml


class TrajectoryReader:
    def __init__(self, file_path):
        self.file_path = file_path
        self.data = None
        self._load_file()

    def _load_file(self):
        """Loads the YAML file and parses its content."""
        try:
            with open(self.file_path, "r") as file:
                self.data = yaml.safe_load(file)
        except FileNotFoundError:
            raise FileNotFoundError(f"File not found: {self.file_path}")
        except yaml.YAMLError as e:
            raise ValueError(f"Error parsing YAML file: {e}")

    def get_header(self):
        """Returns the header information."""
        if self.data and "header" in self.data:
            return self.data["header"]
        return None

    def get_poses(self):
        """Returns the list of poses."""
        if self.data and "poses" in self.data:
            return self.data["poses"]
        return None

    def get_pose_by_index(self, index):
        """Returns a specific pose by its index."""
        poses = self.get_poses()
        if poses and 0 <= index < len(poses):
            return poses[index]
        raise IndexError(f"Pose index {index} is out of range.")

    def get_closest_pose_by_timestamp(self, timestamp):
        """Returns the pose with the closest timestamp."""
        poses = self.get_poses()
        if not poses:
            raise ValueError("No poses available in the data.")

        closest_pose = min(poses, key=lambda pose: abs(pose["stamp"] - timestamp))
        return {
            "position": closest_pose["position"],
            "orientation": closest_pose["orientation"],
        }


# Example usage:
# reader = TrajectoryReader('path_to_file.yaml')
# header = reader.get_header()
# poses = reader.get_poses()
# specific_pose = reader.get_pose_by_index(0)
