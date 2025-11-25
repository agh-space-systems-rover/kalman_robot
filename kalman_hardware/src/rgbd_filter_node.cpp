#include <opencv2/opencv.hpp>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

#include <image_transport/camera_common.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace kalman_hardware {

class RgbdFilter : public rclcpp::Node {
public:
	image_transport::SubscriberFilter                         color_sub;
	image_transport::SubscriberFilter                         depth_sub;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub;
	std::shared_ptr<message_filters::TimeSynchronizer<
	    sensor_msgs::msg::Image,
	    sensor_msgs::msg::Image,
	    sensor_msgs::msg::CameraInfo>>
	    sync;

	image_transport::Publisher                                 color_pub;
	image_transport::Publisher                                 depth_pub;
	rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;

	rclcpp::Time last_stamp_time;

	RgbdFilter(const rclcpp::NodeOptions &options)
	    : Node("rgbd_throttle", options) {
		// Declare parameters
		declare_parameter("color_transport", "raw");
		declare_parameter("depth_transport", "raw");
		declare_parameter("max_rate", 15.0);
		declare_parameter("max_height", 360);

		// Setup topics
		std::string color_in =
		    get_node_base_interface()->resolve_topic_or_service_name(
		        "in/color/image_raw", false, false
		    );
		std::string depth_in =
		    get_node_base_interface()->resolve_topic_or_service_name(
		        "in/depth/image_raw", false, false
		    );
		std::string info_in = image_transport::getCameraInfoTopic(color_in);

		std::string color_out =
		    get_node_base_interface()->resolve_topic_or_service_name(
		        "out/color/image_raw", false, false
		    );
		std::string depth_out =
		    get_node_base_interface()->resolve_topic_or_service_name(
		        "out/depth/image_raw", false, false
		    );
		std::string info_out = image_transport::getCameraInfoTopic(color_out);

		// Setup subscribers
		rclcpp::QoS sub_qos = rclcpp::QoS(1);
		sub_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
		sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
		rclcpp::QoS pub_qos = rclcpp::QoS(1);
		pub_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
		pub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
		// ^ Falls back to best-effort if subscriber is not reliable.

		color_sub.subscribe(
		    this,
		    color_in,
		    image_transport::TransportHints(this, "raw", "color_transport")
		        .getTransport(),
		    sub_qos.get_rmw_qos_profile()
		);
		depth_sub.subscribe(
		    this,
		    depth_in,
		    image_transport::TransportHints(this, "raw", "depth_transport")
		        .getTransport(),
		    sub_qos.get_rmw_qos_profile()
		);
		info_sub.subscribe(this, info_in);

		// Setup publishers
		color_pub = image_transport::create_publisher(
		    this, color_out, pub_qos.get_rmw_qos_profile()
		);
		depth_pub = image_transport::create_publisher(
		    this, depth_out, pub_qos.get_rmw_qos_profile()
		);
		info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(info_out, 10);

		// Setup synchronizer
		sync = std::make_shared<message_filters::TimeSynchronizer<
		    sensor_msgs::msg::Image,
		    sensor_msgs::msg::Image,
		    sensor_msgs::msg::CameraInfo>>(color_sub, depth_sub, info_sub, 10);
		sync->registerCallback(std::bind(
		    &RgbdFilter::callback,
		    this,
		    std::placeholders::_1,
		    std::placeholders::_2,
		    std::placeholders::_3
		));
	}

	void callback(
	    const sensor_msgs::msg::Image::ConstSharedPtr      &color_msg,
	    const sensor_msgs::msg::Image::ConstSharedPtr      &depth_msg,
	    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg
	) {
		// Rate limit
		rclcpp::Duration max_dt = rclcpp::Duration::from_seconds(
		    1.0 / get_parameter("max_rate").as_double()
		);
		rclcpp::Time stamp_time = rclcpp::Time(color_msg->header.stamp);
		if (last_stamp_time.seconds() != 0 &&
		    last_stamp_time.nanoseconds() != 0 &&
		    stamp_time - last_stamp_time < max_dt) {
			return;
		}
		last_stamp_time = stamp_time;

		// Process and publish images
		size_t max_height = get_parameter("max_height").as_int();
		size_t max_width  = color_msg->width * max_height / color_msg->height;

		// Handle color image
		if (color_pub.getNumSubscribers() > 0) {
			if (color_msg->height > max_height) {
				cv_bridge::CvImagePtr color_cv = cv_bridge::toCvCopy(color_msg);
				cv::resize(
				    color_cv->image,
				    color_cv->image,
				    cv::Size(max_width, max_height),
				    0,
				    0,
				    cv::INTER_AREA
				);

				auto new_color_msg    = color_cv->toImageMsg();
				new_color_msg->header = color_msg->header;
				color_pub.publish(new_color_msg);
			} else {
				color_pub.publish(color_msg);
			}
		}

		// Handle depth image
		if (depth_pub.getNumSubscribers() > 0) {
			if (depth_msg->height > max_height) {
				cv_bridge::CvImagePtr depth_cv = cv_bridge::toCvCopy(depth_msg);
				cv::resize(
				    depth_cv->image,
				    depth_cv->image,
				    cv::Size(max_width, max_height),
				    0,
				    0,
				    cv::INTER_NEAREST
				);
				auto new_depth_msg    = depth_cv->toImageMsg();
				new_depth_msg->header = depth_msg->header;
				depth_pub.publish(new_depth_msg);
			} else {
				depth_pub.publish(depth_msg);
			}
		}

		// Publish camera info
		// Need to scale projection matrices if image is resized.
		if (info_pub->get_subscription_count() > 0) {
			if (color_msg->height > max_height) {
				auto new_info_msg =
				    std::make_shared<sensor_msgs::msg::CameraInfo>(*info_msg);
				double scale =
				    static_cast<double>(max_height) / color_msg->height;

				new_info_msg->k[0] *= scale; // fx
				new_info_msg->k[2] *= scale; // cx
				new_info_msg->k[4] *= scale; // fy
				new_info_msg->k[5] *= scale; // cy

				new_info_msg->p[0] *= scale; // fx
				new_info_msg->p[2] *= scale; // cx
				new_info_msg->p[5] *= scale; // fy
				new_info_msg->p[6] *= scale; // cy

				new_info_msg->width  = max_width;
				new_info_msg->height = max_height;

				info_pub->publish(*new_info_msg);
			} else {
				info_pub->publish(*info_msg);
			}
		}
	}
};

} // namespace kalman_hardware

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_hardware::RgbdFilter)
