import os
import sys
from gui.image_viewer import ImageViewer
from utils.timestamp import extract_timestamp
from utils.trajectory import TrajectoryReader


def main(folder_path, trajectory: TrajectoryReader, out_folder):
    if not os.path.isdir(folder_path):
        print(f"The provided path '{folder_path}' is not a valid directory.")
        sys.exit(1)

    viewer = ImageViewer(folder_path, trajectory, out_folder)
    viewer.run()


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python main.py <folder_path> <trajectory_file.yaml> <out_folder>")
        sys.exit(1)

    out_folder = os.path.join(os.getcwd(), sys.argv[3])
    if not os.path.exists(out_folder):
        os.makedirs(out_folder)

    reader = TrajectoryReader(sys.argv[2])
    header = reader.get_header()
    poses = reader.get_poses()
    specific_pose = reader.get_pose_by_index(0)
    # print(header)
    # print(poses)
    print(specific_pose)

    folder_path = sys.argv[1]
    main(folder_path, reader, out_folder)
