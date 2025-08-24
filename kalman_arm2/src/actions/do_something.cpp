#include "actions/do_something.hpp"

DoSomething::DoSomething(
    const std::string           &name,
    const BT::NodeConfiguration &config,
    rclcpp::Node                *parent
)
    : BT::SyncActionNode(name, config), parent_{parent} {}

BT::PortsList DoSomething::providedPorts()
{
return { BT::InputPort<std::string>("target") };
}

BT::NodeStatus DoSomething::tick() {
	auto target = getInput<std::string>("target").value_or("world");
	RCLCPP_INFO(
	    parent_->get_logger(), "[DoSomething] working on %s", target.c_str()
	);
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
	return BT::NodeStatus::SUCCESS;
}