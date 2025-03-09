#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/path.hpp>

#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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
		path_sub.subscribe(this, "/rtabmap/pathMap");
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


        pcl::io::savePCDFileASCII("combined_cloud.pcd", *cloud);

    }

};
} // namespace kalman_arch

 RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arch::SlamSerialization)