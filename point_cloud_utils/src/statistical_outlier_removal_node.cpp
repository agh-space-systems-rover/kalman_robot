#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

constexpr int   default_queue_size = 10;
constexpr float default_leaf_size  = 0.1;

namespace point_cloud_utils {

class StatisticalOutlierRemoval : public rclcpp::Node {
  public:
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub;
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub;
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB>               filter;

	StatisticalOutlierRemoval(const rclcpp::NodeOptions &options)
	    : Node("statistical_outlier_removal", options) {
		// Declare parameters.
		declare_parameter("queue_size", default_queue_size);
		declare_parameter("num_neighbors", 10);
		declare_parameter("std_dev_mul", 1.0);

		// Read static parameters.
		int queue_size = default_queue_size;
		get_parameter("queue_size", queue_size);

		// Create a publisher.
		pub = create_publisher<sensor_msgs::msg::PointCloud2>(
		    "output", queue_size
		);

		// Create a subscriber.
		sub = create_subscription<sensor_msgs::msg::PointCloud2>(
		    "input",
		    queue_size,
		    std::bind(
		        &StatisticalOutlierRemoval::callback,
		        this,
		        std::placeholders::_1
		    )
		);
	}

	void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
		// Read dynamic parameters.
		int   num_neighbors = get_parameter("num_neighbors").as_int();
		float std_dev_mul   = get_parameter("std_dev_mul").as_double();

		// Convert the message to a PCL point cloud.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr
		    cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*msg, *cloud);

		// Run the filter.
		if (cloud->size() > 0) {
			filter.setInputCloud(cloud);
			filter.setMeanK(num_neighbors);
			filter.setStddevMulThresh(std_dev_mul);
			filter.filter(*cloud);
		}

		// Convert the point cloud back to a message.
		pcl::toROSMsg(*cloud, *msg);

		// Publish the message.
		pub->publish(*msg);
	}
};

} // namespace point_cloud_utils

RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_utils::StatisticalOutlierRemoval)
