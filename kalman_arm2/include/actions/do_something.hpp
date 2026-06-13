#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>
#include <thread>

class DoSomething : public BT::SyncActionNode {
public:
	DoSomething(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus tick() override;

private:
	rclcpp::Node *parent_;
};
