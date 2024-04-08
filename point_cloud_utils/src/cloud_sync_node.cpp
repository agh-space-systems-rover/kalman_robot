#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>

#include <any>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ApproximateTime

template <size_t N, typename... Args> struct approx_policy_n {
	using type = typename approx_policy_n<
	    N - 1,
	    sensor_msgs::msg::PointCloud2,
	    Args...>::type;
};

template <typename... Args> struct approx_policy_n<0, Args...> {
	using type = message_filters::sync_policies::ApproximateTime<Args...>;
};

// ExactTime

template <size_t N, typename... Args> struct exact_policy_n {
	using type =
	    typename exact_policy_n<N - 1, sensor_msgs::msg::PointCloud2, Args...>::
	        type;
};

template <typename... Args> struct exact_policy_n<0, Args...> {
	using type = message_filters::sync_policies::ExactTime<Args...>;
};

namespace point_cloud_utils {

class CloudSync : public rclcpp::Node {
  public:
	// parameters
	int         number_of_inputs = 0;
	int         queue_size       = 10;
	bool        approx_sync      = true;
	std::string output_frame_id  = "base_link";

	std::vector<std::shared_ptr<
	    message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>>
	         subscribers;
	std::any synchronizer_keepalive;
	std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>>
	    subscriber;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
	         point_cloud_publisher;
	std::any sync_callback_keepalive;

	tf2_ros::Buffer::SharedPtr                  tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	sensor_msgs::msg::PointCloud2::SharedPtr    output_cloud;
	sensor_msgs::msg::PointCloud2::SharedPtr    transformed_cloud;
	rclcpp::Duration                            pc_stamp_offset_sum;
	size_t                                      pc_stamp_offset_count;

	CloudSync(const rclcpp::NodeOptions &options)
	    : Node("cloud_sync", options), pc_stamp_offset_sum(0, 0) {
		// Declare parameters.
		declare_parameter("number_of_inputs", number_of_inputs);
		declare_parameter("approx_sync", approx_sync);
		declare_parameter("queue_size", queue_size);
		declare_parameter("output_frame_id", output_frame_id);

		// Read static parameters.
		get_parameter("number_of_inputs", number_of_inputs);
		get_parameter("approx_sync", approx_sync);
		get_parameter("queue_size", queue_size);

		// Create a TF buffer.
		tf_buffer   = std::make_shared<tf2_ros::Buffer>(get_clock());
		tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

		// Create a publisher for the merged point cloud.
		point_cloud_publisher =
		    create_publisher<sensor_msgs::msg::PointCloud2>("output", 10);

		// Create the output point clouds.
		output_cloud      = std::make_shared<sensor_msgs::msg::PointCloud2>();
		transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

		switch (number_of_inputs) {
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
			    "Invalid number of input topics: " +
			    std::to_string(number_of_inputs)
			);
		}
	}

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
			    in->header.frame_id,
			    rclcpp::Time(0)
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

		// Update the timestamp of the output point cloud.
		pc_stamp_offset_sum =
		    pc_stamp_offset_sum +
		    (static_cast<rclcpp::Time>(in->header.stamp) -
		     static_cast<rclcpp::Time>(output_cloud->header.stamp));
		pc_stamp_offset_count++;

		if constexpr (sizeof...(other_in) > 0) {
			concat_clouds(other_in...);
		}

		return true;
	}

	template <typename... Args> struct sync_callback_t {
		CloudSync *node;

		sync_callback_t(CloudSync *node) : node(node) {}

		void sync_callback(Args... point_clouds) {
			// Read dynamic parameters.
			node->get_parameter("output_frame_id", node->output_frame_id);

			// Update the header of the output point cloud.
			// Stamp is a summy value later changed in concat_clouds.
			node->output_cloud->header.stamp    = node->get_clock()->now();
			node->output_cloud->header.frame_id = node->output_frame_id;

			// Clear the output point cloud.
			node->output_cloud->data.clear();
			node->output_cloud->width      = 0;
			node->output_cloud->height     = 0;
			node->output_cloud->row_step   = 0;
			node->output_cloud->point_step = 0;

			// Merge inputs into the output cloud.
			node->pc_stamp_offset_sum   = rclcpp::Duration(0, 0);
			node->pc_stamp_offset_count = 0;
			if (node->concat_clouds(point_clouds...)) {
				// Fix row_step
				node->output_cloud->row_step = node->output_cloud->data.size();

				// Average the timestamp offset.
				rclcpp::Time new_stamp  = node->output_cloud->header.stamp;
				new_stamp              += node->pc_stamp_offset_sum *
				             (1.0 / node->pc_stamp_offset_count);
				node->output_cloud->header.stamp = new_stamp;

				// Publish the merged point cloud.
				node->point_cloud_publisher->publish(*(node->output_cloud));
			}
		}
	};

	template <size_t N, typename... Args> struct sync_callback_n {
		using type = typename sync_callback_n<
		    N - 1,
		    const sensor_msgs::msg::PointCloud2::SharedPtr &,
		    Args...>::type;
	};

	template <typename... Args> struct sync_callback_n<0, Args...> {
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

	template <typename SyncPolicy, size_t N> void run_with_sync_policy() {
		// Create a subscriber for every point cloud.
		for (int i = 0; i < number_of_inputs; i++) {
			subscribers.push_back(
			    std::make_shared<
			        message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>(
			        this, std::string("input") + std::to_string(i)
			    )
			);
		}

		// Create a synchronizer for the point clouds.
		// ApproximateTime synchronizer will wait for the point clouds to be
		// within a certain time threshold. ExactTime synchronizer will wait for
		// the point clouds to be exactly at the same time.
		auto synchronizer_typed = create_synchronizer<SyncPolicy, N>();

		// Register the callback function to be called when all point clouds are
		// received.
		using sync_callback_type = typename sync_callback_n<N>::type;
		auto sync_callback       = std::make_shared<sync_callback_type>(this);
		sync_callback_keepalive  = sync_callback;
		synchronizer_typed->registerCallback(
		    &sync_callback_type::sync_callback, sync_callback.get()
		);

		// Save the synchronizer to std::any so it doesn't get destroyed.
		synchronizer_keepalive = synchronizer_typed;
	}

	template <size_t N> void run_with_n_topics() {
		if (approx_sync) {
			using approx_policy = typename approx_policy_n<N>::type;
			run_with_sync_policy<approx_policy, N>();
		} else {
			using exact_policy = typename exact_policy_n<N>::type;
			run_with_sync_policy<exact_policy, N>();
		}
	}

	void run_for_single_topic() {
		// Create the callback.
		using sync_callback_type =
		    sync_callback_t<sensor_msgs::msg::PointCloud2::SharedPtr>;
		auto sync_callback      = sync_callback_type(this);
		sync_callback_keepalive = sync_callback;

		// Create a subscriber for the point cloud.
		subscriber = create_subscription<sensor_msgs::msg::PointCloud2>(
		    "input",
		    queue_size,
		    std::bind(
		        &sync_callback_type::sync_callback,
		        sync_callback,
		        std::placeholders::_1
		    )
		);
	}
};

} // namespace point_cloud_utils

RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_utils::CloudSync)
