#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	rclcpp::Node::SharedPtr ros_node = std::make_shared<rclcpp::Node>("voxel_grid");

	// Declare parameters.
	ros_node->declare_parameter("approx_sync", true);
	ros_node->declare_parameter("queue_size", 10);
	ros_node->declare_parameter("leaf_size", 0.1);

	// Read parameters.
	bool approx_sync;
	int queue_size;
	float leaf_size;
	ros_node->get_parameter("approx_sync", approx_sync);
	ros_node->get_parameter("queue_size", queue_size);
	ros_node->get_parameter("leaf_size", leaf_size);

	// Create a publisher.
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = ros_node->create_publisher<sensor_msgs::msg::PointCloud2>("output", queue_size);

	// Create a subscriber.
	auto sub = ros_node->create_subscription<sensor_msgs::msg::PointCloud2>("input", queue_size, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
		// Convert the message to a PCL point cloud.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*msg, *cloud);

		// Run the voxel grid filter.
		pcl::VoxelGrid<pcl::PointXYZRGB> filter;
		filter.setInputCloud(cloud);
		filter.setLeafSize(leaf_size, leaf_size, leaf_size);
		filter.filter(*cloud);

		// Convert the point cloud back to a message.
		pcl::toROSMsg(*cloud, *msg);

		// Publish the message.
		pub->publish(*msg);
	});

	rclcpp::spin(ros_node);

	return 0;
}
