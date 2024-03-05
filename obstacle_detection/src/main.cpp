#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);

	rclcpp::Node::SharedPtr ros_node = std::make_shared<rclcpp::Node>("obstacle_detection");
	tf2_ros::Buffer::SharedPtr tf_buffer   = std::make_shared<tf2_ros::Buffer>(ros_node->get_clock());
	auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

	// Declare parameters.
	ros_node->declare_parameter("approx_sync", true);
	ros_node->declare_parameter("queue_size", 10);
	ros_node->declare_parameter("world_frame", "odom");
	ros_node->declare_parameter("normal_estimation_radius", 0.2);
	ros_node->declare_parameter("max_ground_angle", 25 * M_PI / 180);
	ros_node->declare_parameter("outlier_removal_radius", 0.2);
	ros_node->declare_parameter("outlier_removal_min_neighbors", 10);

	// Read parameters.
	bool approx_sync;
	int queue_size;
	std::string world_frame;
	float normal_estimation_radius;
	float max_ground_angle;
	float outlier_removal_radius;
	int outlier_removal_min_neighbors;
	ros_node->get_parameter("approx_sync", approx_sync);
	ros_node->get_parameter("queue_size", queue_size);
	ros_node->get_parameter("world_frame", world_frame);
	ros_node->get_parameter("normal_estimation_radius", normal_estimation_radius);
	ros_node->get_parameter("max_ground_angle", max_ground_angle);
	ros_node->get_parameter("outlier_removal_radius", outlier_removal_radius);
	ros_node->get_parameter("outlier_removal_min_neighbors", outlier_removal_min_neighbors);

	// Create a publisher.
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub = ros_node->create_publisher<sensor_msgs::msg::PointCloud2>("output", queue_size);

	// Create a subscriber.
	auto sub = ros_node->create_subscription<sensor_msgs::msg::PointCloud2>("input", queue_size, [&](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {		
		// Convert the message to a PCL point cloud.
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::fromROSMsg(*msg, *cloud);

		// Only continue if there are enough points to achieve anything.
		if (cloud->size() >= static_cast<size_t>(outlier_removal_min_neighbors)) {
			// Estimate normals.
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
			ne.setInputCloud(cloud);
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
			ne.setSearchMethod(tree);
			pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			ne.setRadiusSearch(normal_estimation_radius);
			ne.compute(*normals);

			// Get the transform from the frame of the point cloud to the world frame.
			geometry_msgs::msg::TransformStamped transform;
			try {
				transform = tf_buffer->lookupTransform(world_frame, msg->header.frame_id, tf2::TimePointZero);
			} catch (tf2::TransformException &ex) {
				RCLCPP_ERROR(ros_node->get_logger(), "%s", ex.what());
				return;
			}
			Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform);

			// // Visualize normals using the color of the points.
			// for (size_t i = 0; i < normals->size(); i++) {
			// 	// Transform the normal to world_frame
			// 	Eigen::Vector3d normal(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
			// 	Eigen::Isometry3d transform_eigen = tf2::transformToEigen(transform);
			// 	normal = transform_eigen.linear() * normal;

			// 	cloud->points[i].r = (normal.x() * 0.5F + 0.5F) * 255;
			// 	cloud->points[i].g = (normal.y() * 0.5F + 0.5F) * 255;
			// 	cloud->points[i].b = (normal.z() * 0.5F + 0.5F) * 255;
			// }

			// Filter points based on the angle between the normal and the z-axis.
			pcl::PointIndices::Ptr obstacles(new pcl::PointIndices);
			for (size_t i = 0; i < normals->size(); i++) {
				// Transform the normal to world_frame
				Eigen::Vector3d normal(normals->points[i].normal_x, normals->points[i].normal_y, normals->points[i].normal_z);
				normal = transform_eigen.linear() * normal;

				if (abs(normal.z()) > cos(max_ground_angle)) {
					obstacles->indices.push_back(i);
				}
			}
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(obstacles);
			extract.setNegative(true);
			extract.filter(*cloud);

			// Remove outliers if there are enough points in the obstacle cloud.
			if (cloud->size() > 0) {
				pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> ror;
				ror.setInputCloud(cloud);
				ror.setRadiusSearch(outlier_removal_radius);
				ror.setMinNeighborsInRadius(outlier_removal_min_neighbors);
				ror.filter(*cloud);
			}
		}

		// Convert the point cloud back to a message.
		pcl::toROSMsg(*cloud, *msg);

		// Publish the message.
		pub->publish(*msg);
	});

	rclcpp::spin(ros_node);

	return 0;
}
