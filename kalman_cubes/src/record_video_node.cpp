#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <filesystem>


class RecordVideo: public rclcpp::Node {
    public:
    // Subscribers
    image_transport::ImageTransport   it;
    image_transport::Subscriber image_sub;
    int codec = cv::VideoWriter::fourcc('H', '2', '6', '4');
    std::string user_home = std::filesystem::path(getenv("HOME")).string();
    std::string filename = user_home + "arch-video001.mkv";
    int frame_rate;
    cv::VideoWriter writer;
    
    RecordVideo(const rclcpp::NodeOptions &options = {})
        : Node("record_video", options),
          it(std::shared_ptr<RecordVideo>(this, [](auto *) {})) {
      frame_rate        = this->declare_parameter("frame_rate", 5);
        cv::Size sizeFrame(640,320);
        get_parameter("frame_rate", frame_rate);
        writer.open(filename, cv::CAP_FFMPEG, codec, frame_rate, sizeFrame, 1);
      auto image_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                           .best_effort()
                           .durability_volatile();

      // Resolve topics.
      std::string input_image_topic =
          get_node_topics_interface()->resolve_topic_name("/d455_front/color/image_raw");
      // Subscribers
      std::string image_transport =
          this->declare_parameter("image_transport", std::string("raw"));
      image_transport::TransportHints hints(this, "raw", "image_transport");
      image_sub = it.subscribe(
          input_image_topic, 1, &RecordVideo::image_cb, this, &hints
      );
    }
    ~RecordVideo(){
        writer.release();
    }
    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        auto now = this->now();
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat resized;
        cv::resize(cv_ptr->image, resized, {640, 320});
        writer.write(resized);
    }
}; 

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RecordVideo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
  return 0;
}
