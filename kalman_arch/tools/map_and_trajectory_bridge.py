import rospy
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
import yaml
import pcl
import sensor_msgs.point_cloud2 as pc2

class MapAndTrajectoryBridge:
    def __init__(self, ply_file_path, yaml_file_path):
        # Initialize the bridge with PLY and YAML file paths
        self.points = self.read_ply_file(ply_file_path)
        self.path_data = self.read_yaml_file(yaml_file_path)

    def read_ply_file(ply_file_path):
        # Read points from a PLY file using the pcl library
        cloud = pcl.load(ply_file_path)
        points = []
        for point in cloud:
            points.append([point[0], point[1], point[2]])
        return points

    def read_yaml_file(yaml_file_path):
        # Read path data from a YAML file
        with open(yaml_file_path, 'r') as file:
            data = yaml.safe_load(file)
            path_data = []
        for entry in data['path']:
            pose = PoseStamped()
            pose.pose.position.x = entry['position']['x']
            pose.pose.position.y = entry['position']['y']
            pose.pose.position.z = entry['position']['z']
            pose.pose.orientation.x = entry['orientation']['x']
            pose.pose.orientation.y = entry['orientation']['y']
            pose.pose.orientation.z = entry['orientation']['z']
            pose.pose.orientation.w = entry['orientation']['w']
            path_data.append(pose)
        return path_data

    def publish_point_cloud(points):
        # Publish point cloud data to the /map topic
        pub = rospy.Publisher('/map', PointCloud2, queue_size=10)
        rospy.init_node('map_and_trajectory_bridge', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)
        ]

        header = rospy.Header()
        header.frame_id = "map"

        while not rospy.is_shutdown():
            # Create and publish the PointCloud2 message
            pc2_msg = pc2.create_cloud(header, fields, points)
            pub.publish(pc2_msg)
            rate.sleep()

    def publish_path(path_data):
        # Publish path data to the /path topic
        pub = rospy.Publisher('/path', Path, queue_size=10)
        rospy.init_node('map_and_trajectory_bridge', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.poses = path_data

        while not rospy.is_shutdown():
            # Publish the Path message
            pub.publish(path_msg)
            rate.sleep()

if __name__ == '__main__':
    # Define file paths for the PLY and YAML files
    ply_file_path = '/path/to/your/map.ply'
    yaml_file_path = '/path/to/your/path.yaml'

    # Read points and path data from the files
    points = read_ply_file(ply_file_path)
    path_data = read_yaml_file(yaml_file_path)

    try:
        # Publish the point cloud and path data
        publish_point_cloud(points)
        publish_path(path_data)
    except rospy.ROSInterruptException:
        pass