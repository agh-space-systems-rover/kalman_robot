#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_common.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace point_cloud_utils {

class RgbdCloud : public rclcpp::Node {
  public:
	image_transport::SubscriberFilter                         color_sub;
	image_transport::SubscriberFilter                         reg_depth_sub;
	message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub;
	std::shared_ptr<message_filters::TimeSynchronizer<
	    sensor_msgs::msg::Image,
	    sensor_msgs::msg::Image,
	    sensor_msgs::msg::CameraInfo>>
	                                                            sync;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub;
	sensor_msgs::msg::PointCloud2                               pc_msg;
	std::mt19937                          rng{std::random_device{}()};
	std::uniform_real_distribution<float> dist{-1, 1};

	RgbdCloud(const rclcpp::NodeOptions &options)
	    : Node("rgbd_cloud", options) {
		// Declare parameters.
		declare_parameter("color_transport", "raw");
		declare_parameter("depth_transport", "raw");
		declare_parameter("cloud_width", 160);
		declare_parameter("min_distance", 0.1);
		declare_parameter("max_distance", 5.0);

		// Create subscribers.
		rclcpp::QoS img_qos = rclcpp::QoS(1);
		img_qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);
		img_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

		std::string color_topic =
		    get_node_base_interface()->resolve_topic_or_service_name(
		        "color/image_raw", false, false
		    );
		std::string depth_topic =
		    get_node_base_interface()->resolve_topic_or_service_name(
		        "depth/image_raw", false, false
		    );
		std::string info_topic =
		    image_transport::getCameraInfoTopic(color_topic);

		color_sub.subscribe(
		    this,
		    color_topic,
		    image_transport::TransportHints(this, "raw", "color_transport")
		        .getTransport(),
		    img_qos.get_rmw_qos_profile()
		);
		reg_depth_sub.subscribe(
		    this,
		    depth_topic,
		    image_transport::TransportHints(this, "raw", "depth_transport")
		        .getTransport(),
		    img_qos.get_rmw_qos_profile()
		);
		info_sub.subscribe(this, info_topic);

		// Create publisher.
		pub = create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);

		// Create time synchronizer.
		sync = std::make_shared<message_filters::TimeSynchronizer<
		    sensor_msgs::msg::Image,
		    sensor_msgs::msg::Image,
		    sensor_msgs::msg::CameraInfo>>(
		    color_sub, reg_depth_sub, info_sub, 10
		);
		sync->registerCallback(std::bind(
		    &RgbdCloud::callback,
		    this,
		    std::placeholders::_1,
		    std::placeholders::_2,
		    std::placeholders::_3
		));
	}

	void callback(
	    const sensor_msgs::msg::Image::ConstSharedPtr      &color,
	    const sensor_msgs::msg::Image::ConstSharedPtr      &depth,
	    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info
	) {
		float  aspect_ratio = static_cast<float>(info->width) / info->height;
		size_t pc_width     = get_parameter("cloud_width").as_int();
		size_t pc_height    = pc_width / aspect_ratio;

		// Reserve space for the point cloud.
		pc_msg.data.reserve(
		    // 4 bytes per float
		    // 4 floats per point (x, y, z, rgb)
		    // rgb is actually uint32.
		    pc_width * pc_height * 4 * 4
		);
		pc_msg.data.clear();

		// Decode using OpenCV.
		cv_bridge::CvImageConstPtr cv_color = cv_bridge::toCvShare(color);
		cv_bridge::CvImageConstPtr cv_depth = cv_bridge::toCvShare(depth);

		// Declare some values.
		float src_scale     = static_cast<float>(info->height) / pc_height;
		float sample_radius = src_scale / 2;
		float cx            = info->p[2];
		float cy            = info->p[6];
		float fx            = info->p[0];
		float fy            = info->p[5];
		float min_distance  = get_parameter("min_distance").as_double();
		float max_distance  = get_parameter("max_distance").as_double();

		for (size_t row = 0; row < pc_height; row++) {
			for (size_t col = 0; col < pc_width; col++) {
				size_t src_row = row * src_scale + 0.5;
				size_t src_col = col * src_scale + 0.5;

				// Sample around the center of the pixel.
				src_row =
				    static_cast<float>(src_row) + dist(rng) * sample_radius;
				src_col =
				    static_cast<float>(src_col) + dist(rng) * sample_radius;

				if (src_row >= info->height || src_col >= info->width) {
					continue;
				}

				uint16_t src_depth =
				    cv_depth->image.at<uint16_t>(src_row, src_col);

				cv::Vec3b color =
				    cv_color->image.at<cv::Vec3b>(src_row, src_col);
				uint8_t src_r = color[0];
				uint8_t src_g = color[1];
				uint8_t src_b = color[2];

				float z = src_depth / 1000.0F;
				if (min_distance <= z && z <= max_distance) {
					float    x   = (src_col - cx) * z / fx;
					float    y   = (src_row - cy) * z / fy;
					uint32_t rgb = (src_r & 0xff) << 16 | (src_g & 0xff) << 8 |
					               (src_b & 0xff);

					// Add a point to the buffer.
					size_t i = pc_msg.data.size();
					pc_msg.data.resize(pc_msg.data.size() + 16);

					// Write the values.
					*reinterpret_cast<float *>(&pc_msg.data[i + 0])     = x;
					*reinterpret_cast<float *>(&pc_msg.data[i + 4])     = y;
					*reinterpret_cast<float *>(&pc_msg.data[i + 8])     = z;
					*reinterpret_cast<uint32_t *>(&pc_msg.data[i + 12]) = rgb;
				}
			}
		}

		// Initialize the rest of the point cloud message.
		pc_msg.header = info->header;
		pc_msg.height = 1;
		pc_msg.width  = pc_msg.data.size() / 16;
		pc_msg.fields.resize(4);
		pc_msg.fields[0].name     = "x";
		pc_msg.fields[0].offset   = 0;
		pc_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
		pc_msg.fields[0].count    = 1;
		pc_msg.fields[1].name     = "y";
		pc_msg.fields[1].offset   = 4;
		pc_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
		pc_msg.fields[1].count    = 1;
		pc_msg.fields[2].name     = "z";
		pc_msg.fields[2].offset   = 8;
		pc_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
		pc_msg.fields[2].count    = 1;
		pc_msg.fields[3].name     = "rgb";
		pc_msg.fields[3].offset   = 12;
		pc_msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
		pc_msg.fields[3].count    = 1;
		pc_msg.is_bigendian       = false;
		pc_msg.point_step         = 16;
		pc_msg.row_step           = pc_msg.data.size();
		pc_msg.is_dense           = true;

		// Publish the point cloud.
		pub->publish(pc_msg);
	}
};

} // namespace point_cloud_utils

RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_utils::RgbdCloud)
