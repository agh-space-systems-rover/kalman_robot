#pragma once

#include <behaviortree_cpp_v3/action_node.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class RotateEEAlongNormalTwist : public BT::StatefulActionNode {
  public:
	RotateEEAlongNormalTwist(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void           onHalted() override;

  private:
	std::optional<tf2::Transform> current_base_to_ee() const;
	void publish_angular_velocity(const tf2::Vector3 &velocity) const;
	void publish_neutral_twist() const;

	rclcpp::Node *parent_;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
	std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	std::string     base_frame_;
	std::string     ee_frame_;
	tf2::Quaternion target_rotation_;
	rclcpp::Time    deadline_;
	double          angular_kp_{1.5};
	double          max_angular_speed_{0.3};
	double          min_angular_speed_{0.05};
	double          tolerance_rad_{0.0174532925};
};
