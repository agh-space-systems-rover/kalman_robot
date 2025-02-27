#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include "std_srvs/srv/empty.hpp"
#include <filesystem>

namespace kalman_cubes {

class RecordVideo: public rclcpp::Node {
    public:
    // Subscribers
    image_transport::ImageTransport   it;
    image_transport::Subscriber image_sub;
    cv::Mat latest_image;
    int codec = cv::VideoWriter::fourcc('H', '2', '6', '4');
    std::string user_home = std::filesystem::path(getenv("HOME")).string();
    std::string camera_no = this->declare_parameter("camera_no", std::string("d455_front"));
    std::string filename = "/home/rafal/" + camera_no +"_arch-video001.mkv";
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
      // Create service 
      service = create_service<std_srvs::srv::Empty>("take_picture", std::bind(&RecordVideo::take_pic, this, std::placeholders::_1, std::placeholders::_2));
    }

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service;

    void image_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
        auto now = this->now();
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat resized;
        cv::resize(cv_ptr->image, resized, {640, 320});
        writer.write(resized);
        latest_image = resized;
    }
    void take_pic(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
    const std::shared_ptr<std_srvs::srv::Empty::Response> res){
        if(!latest_image.empty()){
        std::string filename = "picture" + this->camera_no + ".jpg";
        cv::imwrite(filename, this->latest_image);
        }
        else{
            RCLCPP_INFO(get_logger(), "Image is empty");
        }
    

    }
    ~RecordVideo(){
        writer.release();
    }
}; 

}

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_cubes::RecordVideo)