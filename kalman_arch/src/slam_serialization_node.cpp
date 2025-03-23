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
	std::filesystem::path user_home = std::filesystem::path(getenv("HOME"));
	std::filesystem::path trajectory_dir = user_home / "arch" / "trajectory";
	std::filesystem::path cloud_dir      = user_home / "arch" / "cloud";
	std::vector<uint64_t> stamp_vector;
	int64_t               first_stamp_us = 0;

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
		// Init stamp
		if (first_stamp_us == 0) {
			first_stamp_us =
			    static_cast<int64_t>(cloud_msg->header.stamp.sec) * 1e6 +
			    static_cast<int64_t>(cloud_msg->header.stamp.nanosec / 1e3);
		}

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		    cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*cloud_msg, *cloud);
		std::string cloud_save_path =
		    (cloud_dir / ("cloud-" + std::to_string(first_stamp) + ".ply"))
		        .string();
		std::filesystem::create_directories(cloud_dir);
		safe_save(cloud_save_path, [&](const std::string &path) {
			pcl::io::savePLYFileBinary(path, *cloud);
		});

		save_yaml_from_msg(path_msg);
	}

	void save_yaml_from_msg(const nav_msgs::msg::Path::ConstSharedPtr &path_msg
	) {
		std::vector<geometry_msgs::msg::PoseStamped> positions;

		for (const auto &pose : path_msg->poses) {
			positions.push_back(pose);
		}

		uint64_t stamp =
		    static_cast<int64_t>(path_msg->header.stamp.sec) * 1e6 +
		    static_cast<int64_t>(path_msg->header.stamp.nanosec / 1e3);

		YAML::Emitter out;
		out << YAML::BeginMap;
		out << YAML::Key << "header" << YAML::Value << YAML::BeginMap;
		out << YAML::Key << "frame_id" << YAML::Value
		    << path_msg->header.frame_id;
		out << YAML::EndMap;

		out << YAML::Key << "poses" << YAML::Value << YAML::BeginSeq;
		for (int i = 0; i < positions.size(); i++) {
			out << YAML::BeginMap;
			if (i < stamp_vector.size()) {
				out << YAML::Key << "stamp" << YAML::Value << stamp_vector[i];
			} else {
				out << YAML::Key << "stamp" << YAML::Value << stamp;
				stamp_vector.push_back(stamp);
			}
			out << YAML::Key << "position" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "x" << YAML::Value
			    << positions[i].pose.position.x;
			out << YAML::Key << "y" << YAML::Value
			    << positions[i].pose.position.y;
			out << YAML::Key << "z" << YAML::Value
			    << positions[i].pose.position.z;
			out << YAML::EndMap;

			out << YAML::Key << "orientation" << YAML::Value << YAML::BeginMap;
			out << YAML::Key << "x" << YAML::Value
			    << positions[i].pose.orientation.x;
			out << YAML::Key << "y" << YAML::Value
			    << positions[i].pose.orientation.y;
			out << YAML::Key << "z" << YAML::Value
			    << positions[i].pose.orientation.z;
			out << YAML::Key << "w" << YAML::Value
			    << positions[i].pose.orientation.w;
			out << YAML::EndMap;
			out << YAML::EndMap;
		}

		out << YAML::Key << YAML::EndSeq;
		out << YAML::Key << YAML::EndMap;

		std::filesystem::create_directories(trajectory_dir);
		std::string trajectory_file_path =
		    (trajectory_dir /
		     ("trajectory-" + std::to_string(first_stamp) + ".yaml"))
		        .string();
		safe_save(trajectory_file_path, [&](const std::string &path) {
			std::ofstream fout(path);
			fout << out.c_str();
		});
	}

	void safe_save(
	    const std::string                       &path,
	    std::function<void(const std::string &)> save_cb
	) {
		// Create directories
		std::filesystem::create_directories(
		    std::filesystem::path(path).parent_path()
		);

		// Save the new file to a temporary location
		std::string new_path = path + ".new";
		if (std::filesystem::exists(new_path)) {
			std::filesystem::remove(new_path);
		}
		save_cb(new_path);

		// Remove the old file while the new one
		// is safe in the temporary location
		if (std::filesystem::exists(path)) {
			std::filesystem::remove(path);
		}

		// Copy-re-move the new file to the final location
		std::filesystem::copy(new_path, path);
		std::filesystem::remove(new_path);
	}
};
} // namespace kalman_arch

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arch::SlamSerialization)