#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>

rclcpp::Node::SharedPtr ros_node;

// parameters
std::vector<std::string> input_topics;
int                      queue_size;
bool                     approx_sync;
std::string              output_topic;
std::string              output_frame_id;
std::string              fixed_frame_id;

std::vector<
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>>
    subscribers;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
                                            point_cloud_publisher;
tf2_ros::Buffer::SharedPtr                  tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
sensor_msgs::msg::PointCloud2::SharedPtr    output_cloud;
sensor_msgs::msg::PointCloud2::SharedPtr    transformed_cloud;

template <typename... Args>
bool concat_clouds(
    const sensor_msgs::msg::PointCloud2::SharedPtr &in, Args... other_in
) {
	// Get transform from sensor frame to base_link.
	// Use odom as the fixed frame for time travel.
	geometry_msgs::msg::TransformStamped transform;
	try {
		transform = tf_buffer->lookupTransform(
		    output_cloud->header.frame_id,
		    output_cloud->header.stamp,
		    in->header.frame_id,
		    in->header.stamp,
		    fixed_frame_id,
		    rclcpp::Duration::from_seconds(1)
		);
	} catch (...) {
		return false;
	}

	// Transform the point cloud to the output frame.
	pcl_ros::transformPointCloud(
	    output_cloud->header.frame_id, transform, *in, *transformed_cloud
	);
	// pcl_ros::transformPointCloud(output_cloud->header.frame_id, *in,
	// *transformed_cloud, *tf_buffer);

	// Concatenate the transformed point cloud to the output point cloud.
	pcl::concatenatePointCloud(
	    *output_cloud, *transformed_cloud, *output_cloud
	);

	if constexpr (sizeof...(other_in) > 0) {
		concat_clouds(other_in...);
	}

	return true;
}

template <typename... Args>
struct sync_callback_t {
	static void sync_callback(Args... point_clouds) {
		// Update the header of the output point cloud.
		output_cloud->header.stamp    = ros_node->get_clock()->now();
		output_cloud->header.frame_id = output_frame_id;

		// Clear the output point cloud.
		output_cloud->data.clear();
		output_cloud->width      = 0;
		output_cloud->height     = 0;
		output_cloud->row_step   = 0;
		output_cloud->point_step = 0;

		// Merge inputs into the output cloud.
		if (concat_clouds(point_clouds...)) {
			// Fix row_step
			output_cloud->row_step = output_cloud->data.size();

			// Publish the merged point cloud.
			point_cloud_publisher->publish(*output_cloud);
		}
	}
};

template <size_t N, typename... Args>
struct sync_callback_n {
	using type = typename sync_callback_n<
	    N - 1,
	    const sensor_msgs::msg::PointCloud2::SharedPtr &,
	    Args...>::type;
};

template <typename... Args>
struct sync_callback_n<0, Args...> {
	using type = sync_callback_t<Args...>;
};

template <typename SyncPolicy, size_t N, typename... Args>
std::shared_ptr<message_filters::Synchronizer<SyncPolicy>>
create_synchronizer(Args &&...args) {
	if constexpr (N == 0) {
		return std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
		    SyncPolicy(queue_size), std::forward<Args>(args)...
		);
	} else {
		return create_synchronizer<SyncPolicy, N - 1>(
		    *subscribers[N - 1], std::forward<Args>(args)...
		);
	}
}

template <typename SyncPolicy, size_t N>
void run_with_sync_policy() {
	// Create a subscriber for every point cloud.
	for (const auto &topic : input_topics) {
		subscribers.push_back(
		    std::make_shared<
		        message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
		        ros_node, topic
		    )
		);
	}

	// Create a synchronizer for the point clouds.
	// ApproximateTime synchronizer will wait for the point clouds to be within
	// a certain time threshold. ExactTime synchronizer will wait for the point
	// clouds to be exactly at the same time.
	auto synchronizer = create_synchronizer<SyncPolicy, N>();

	// Register the callback function to be called when all point clouds are
	// received.
	synchronizer->registerCallback(sync_callback_n<N>::type::sync_callback);

	// Spin the node.
	rclcpp::spin(ros_node);
}

template <size_t N, typename... Args>
struct approx_policy_n {
	using type = typename approx_policy_n<
	    N - 1,
	    sensor_msgs::msg::PointCloud2,
	    Args...>::type;
};

template <typename... Args>
struct approx_policy_n<0, Args...> {
	using type = message_filters::sync_policies::ApproximateTime<Args...>;
};

template <size_t N, typename... Args>
struct exact_policy_n {
	using type =
	    typename exact_policy_n<N - 1, sensor_msgs::msg::PointCloud2, Args...>::
	        type;
};

template <typename... Args>
struct exact_policy_n<0, Args...> {
	using type = message_filters::sync_policies::ExactTime<Args...>;
};

template <size_t N>
void run_with_n_topics() {
	if (approx_sync) {
		using approx_policy = typename approx_policy_n<N>::type;
		run_with_sync_policy<approx_policy, N>();
	} else {
		using exact_policy = typename exact_policy_n<N>::type;
		run_with_sync_policy<exact_policy, N>();
	}
}

void run_for_single_topic() {
	// Create a subscriber for the point cloud.
	auto subscriber =
	    ros_node->create_subscription<sensor_msgs::msg::PointCloud2>(
	        input_topics[0],
	        queue_size,
	        sync_callback_t<
	            const sensor_msgs::msg::PointCloud2::SharedPtr>::sync_callback
	    );

	// Spin the node.
	rclcpp::spin(ros_node);
}

int main(int argc, char **argv) {
	// Create a ROS node and TF buffer.
	rclcpp::init(argc, argv);
	ros_node    = std::make_shared<rclcpp::Node>("point_cloud_sync");
	tf_buffer   = std::make_shared<tf2_ros::Buffer>(ros_node->get_clock());
	tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

	// Declare parameters.
	ros_node->declare_parameter("approx_sync", true);
	ros_node->declare_parameter(
	    "output_topic", std::string{"/point_cloud_sync/points"}
	);
	ros_node->declare_parameter("queue_size", 10);
	ros_node->declare_parameter("input_topics", std::vector<std::string>{});
	ros_node->declare_parameter("output_frame_id", std::string{"base_link"});
	ros_node->declare_parameter("fixed_frame_id", std::string{"odom"});

	// Read parameters.
	ros_node->get_parameter("approx_sync", approx_sync);
	ros_node->get_parameter("output_topic", output_topic);
	ros_node->get_parameter("queue_size", queue_size);
	ros_node->get_parameter("input_topics", input_topics);
	ros_node->get_parameter("output_frame_id", output_frame_id);
	ros_node->get_parameter("fixed_frame_id", fixed_frame_id);

	RCLCPP_INFO(
	    ros_node->get_logger(),
	    "Starting point_cloud_sync node:\n\tapprox_sync: %s\n\toutput_topic: "
	    "%s\n\tqueue_size: %d\n\tinput_topics: %s\n\toutput_frame_id: "
	    "%s\n\tfixed_frame_id: %s",
	    approx_sync ? "true" : "false",
	    output_topic.c_str(),
	    queue_size,
	    std::accumulate(
	        input_topics.begin(),
	        input_topics.end(),
	        std::string{},
	        [](const std::string &a, const std::string &b) {
		        return a + " " + b;
	        }
	    ).c_str(),
	    output_frame_id.c_str(),
	    fixed_frame_id.c_str()
	);

	// Create a publisher for the merged point cloud.
	point_cloud_publisher =
	    ros_node->create_publisher<sensor_msgs::msg::PointCloud2>(
	        output_topic, 10
	    );

	// Create the output point clouds.
	output_cloud      = std::make_shared<sensor_msgs::msg::PointCloud2>();
	transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

	size_t n = input_topics.size();
	switch (n) {
	case 1:
		run_for_single_topic();
		break;
	case 2:
		run_with_n_topics<2>();
		break;
	case 3:
		run_with_n_topics<3>();
		break;
	case 4:
		run_with_n_topics<4>();
		break;
	case 5:
		run_with_n_topics<5>();
		break;
	case 6:
		run_with_n_topics<6>();
		break;
	case 7:
		run_with_n_topics<7>();
		break;
	case 8:
		run_with_n_topics<8>();
		break;
	default:
		throw std::runtime_error(
		    "Invalid number of input topics: " + std::to_string(n)
		);
	}

	return 0;
}
