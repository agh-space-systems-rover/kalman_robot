#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <filesystem>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
#include <yaml-cpp/emitter.h>
#include <yaml-cpp/emitterdef.h>
#include <yaml-cpp/emittermanip.h>
#include <yaml-cpp/yaml.h>

namespace kalman_arch {

class SlamSerialization : public rclcpp::Node {
  public:
	message_filters::Subscriber<sensor_msgs::msg::PointCloud2> cloud_sub;
	message_filters::Subscriber<nav_msgs::msg::Path>           path_sub;
	std::shared_ptr<message_filters::TimeSynchronizer<
	    sensor_msgs::msg::PointCloud2,
	    nav_msgs::msg::Path>>
	    sync;

	SlamSerialization(const rclcpp::NodeOptions &options)
	    : Node("slam_serialization", options) {
		// Subscribers
		cloud_sub.subscribe(this, "/rtabmap/cloud_map");
		path_sub.subscribe(this, "/rtabmap/mapPath");
		// Synchronize messages
		sync = std::make_shared<message_filters::TimeSynchronizer<
		    sensor_msgs::msg::PointCloud2,
		    nav_msgs::msg::Path>>(cloud_sub, path_sub, 10);
		sync->registerCallback(std::bind(
		    &SlamSerialization::callback,
		    this,
		    std::placeholders::_1,
		    std::placeholders::_2
		));
	}
	void callback(
	    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
	    const nav_msgs::msg::Path::ConstSharedPtr           &path_msg
	) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		    cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*cloud_msg, *cloud);
		// pcl::io::savePCDFileASCII("/home/rafal/test_cloud.pcd", *cloud);
		pcl::io::savePLYFile("/home/rafal/cloud.ply", *cloud);
		std::vector<geometry_msgs::msg::PoseStamped> positions;
		std::vector<int> stamp_vector;
		for (const auto &pose : path_msg->poses) {
			positions.push_back(pose);
		}
		
		YAML::Emitter out;
		out << YAML::BeginMap;
		out << YAML::Key << "header" << YAML::Value << YAML::BeginMap;
		out << YAML::Key << "frame_id" << YAML::Value
		    << path_msg->header.frame_id;
		out << YAML::EndMap;

		out << YAML::Key << "poses" << YAML::Value << YAML::BeginSeq;
		for (const auto stamp : stamp_vector ) {
			out << YAML::BeginMap;
			out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "stamp" << YAML::Value << stamp;
			out << YAML::Key << "x" << YAML::Value << positions[stamp].pose.position.x;
			out << YAML::Key << "y" << YAML::Value << positions[stamp].pose.position.y;
			out << YAML::Key << "z" << YAML::Value << positions[stamp].pose.position.z;
			out << YAML::EndMap;

			out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "x" << YAML::Value << positions[stamp].pose.orientation.x;
			out << YAML::Key << "y" << YAML::Value << positions[stamp].pose.orientation.y;
			out << YAML::Key << "z" << YAML::Value << positions[stamp].pose.orientation.z;
			out << YAML::Key << "w" << YAML::Value << positions[stamp].pose.orientation.w;
			out << YAML::EndMap;
			out << YAML::EndMap;
		}

		out << YAML::Key << YAML::EndSeq;
		out << YAML::Key << YAML::EndMap;
		std::ofstream fout("/home/rafal/positions.yaml");
		fout << out.c_str();
	}
};
} // namespace kalman_arch

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arch::SlamSerialization)