#include <filesystem>
#include <functional>
#include <memory>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <cob_srvs/srv/set_int.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <std_msgs/msg/string.hpp>

#ifdef ROS_HUMBLE
#include <cv_bridge/cv_bridge.h>
#else
#include <cv_bridge/cv_bridge.hpp>
#endif

namespace kalman_arch {

class RecordVideo : public rclcpp::Node {
  public:
	// Subscribers
	image_transport::ImageTransport it;
	image_transport::Subscriber     image_sub;
	cv::Mat                         original_latest_image;
	int                             frame_rate;
	int                             single_frame_time;
	int                             old_frame_timestamp = 0;
	int64_t                         last_image_timestamp_ms;
	int codec = cv::VideoWriter::fourcc('H', '2', '6', '4');
	std::unique_ptr<cv::VideoWriter> writer;
	std::filesystem::path user_home = std::filesystem::path(getenv("HOME"));
	std::filesystem::path video_dir = user_home / "arch" / "arch-videos";
	std::filesystem::path screenshots_dir =
	    user_home / "arch" / "arch-screenshots";
	std::string camera_name           = "";
	std::string screenshots_save_path = "";
	int         width, height; // parameter

	RecordVideo(const rclcpp::NodeOptions &options)
	    : Node("record_video", options),
	      it(std::shared_ptr<RecordVideo>(this, [](auto *) {})) {

		frame_rate        = declare_parameter("frame_rate", 5);
		single_frame_time = 1000 / frame_rate;
		camera_name =
		    declare_parameter("camera_name", std::string("d455_front"));
		width  = declare_parameter("video_width", 640);
		height = declare_parameter("video_height", 360);

		screenshots_save_path = (screenshots_dir).string();
		// Resolve topics.
		std::string input_image_topic =
		    get_node_topics_interface()->resolve_topic_name("image_raw");
		// Image subscription
		auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
		                     .best_effort()
		                     .durability_volatile();
		this->declare_parameter("image_transport", std::string("raw"));
		image_transport::TransportHints hints(this, "raw", "image_transport");
		image_sub = image_transport::create_subscription(
		    this,
		    input_image_topic,
		    std::bind(&RecordVideo::image_cb, this, std::placeholders::_1),
		    hints.getTransport(),
		    image_qos.get_rmw_qos_profile()
		);
		// Create service
		service = create_service<cob_srvs::srv::SetInt>(
		    "take_picture",
		    std::bind(
		        &RecordVideo::take_pic,
		        this,
		        std::placeholders::_1,
		        std::placeholders::_2
		    )
		);
	}

	rclcpp::Service<cob_srvs::srv::SetInt>::SharedPtr service;

	void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
		last_image_timestamp_ms =
		    static_cast<int64_t>(msg->header.stamp.sec) * 1e6 +
		    static_cast<int64_t>(msg->header.stamp.nanosec / 1e3);
		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        original_latest_image = cv_ptr->image;

		if (!writer) {
			std::string video_save_path =
			    (video_dir /
			     (camera_name + "-" + std::to_string(last_image_timestamp_ms) +
			      "-video.mkv"))
			        .string();
			std::filesystem::create_directories(video_dir);
			writer = std::make_unique<cv::VideoWriter>();
			cv::Size sizeFrame(width, height);
			writer->open(
			    video_save_path, cv::CAP_FFMPEG, codec, frame_rate, sizeFrame, 1
			);
		}
		if ((last_image_timestamp_ms - old_frame_timestamp) >=
		    single_frame_time) {
            cv::Mat resized;
            cv::resize(cv_ptr->image, resized, {width, height});
			writer->write(resized);
			old_frame_timestamp = last_image_timestamp_ms;
		}

	}
	void take_pic(
	    const std::shared_ptr<cob_srvs::srv::SetInt::Request>  req,
	    const std::shared_ptr<cob_srvs::srv::SetInt::Response> res
	) {
		if (!original_latest_image.empty()) {
			cv::putText(
			    original_latest_image,
			    std::to_string(req->data),
			    cv::Point(10, original_latest_image.rows / 2),
			    cv::FONT_HERSHEY_DUPLEX,
			    1.0,
			    CV_RGB(0, 0, 0)
			);
			std::string filename =
			    screenshots_save_path + "/" +
			    (std::to_string(req->data) + "-" + camera_name + "-" +
			     std::to_string(last_image_timestamp_ms) + "-screenshot.jpg");
			std::filesystem::create_directories(screenshots_dir);
			cv::imwrite(filename, this->original_latest_image);
		} else {
			RCLCPP_INFO(get_logger(), "Image is empty");
		}
	}
	~RecordVideo() {
		writer.release();
	}
};

} // namespace kalman_arch

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arch::RecordVideo)
