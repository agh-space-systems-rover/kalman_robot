#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <kalman_interfaces/msg/arm_values.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>

class SetJaw : public BT::StatefulActionNode {
  public:
	SetJaw(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void           onHalted() override;

  private:
	rclcpp::Node         *parent_;
	double                target_jaw_angle = 0.0;
	std::optional<double> current_jaw_angle{};

	std::shared_ptr<rclcpp::Publisher<kalman_interfaces::msg::ArmValues>> pub_;
	std::shared_ptr<rclcpp::Subscription<kalman_interfaces::msg::ArmValues>>
	             sub_;
	rclcpp::Time start_time_;

	void arm_callback(const kalman_interfaces::msg::ArmValues::SharedPtr msg);
};
