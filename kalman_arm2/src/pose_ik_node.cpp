#include <geometry_msgs/msg/pose.hpp>
#include <kalman_interfaces/action/pose_ik_navigate_to_pose.hpp>
#include <kalman_interfaces/msg/arm_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/msg/string.hpp>
#include <urdf/model.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace kalman_arm2 {

class PoseIK : public rclcpp::Node {
  public:
	using Action = kalman_interfaces::action::PoseIKNavigateToPose;
	using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

	static constexpr const char *ARM_LINK_END = "arm_link_end";
	static constexpr const char *ARM_LINK_GRIPPER = "arm_link_gripper";

	rclcpp::Subscription<kalman_interfaces::msg::ArmValues>::SharedPtr
	    joint_pos_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
	    robot_description_sub;
	rclcpp::Publisher<kalman_interfaces::msg::ArmValues>::SharedPtr joint_vel_pub;
	rclcpp_action::Server<Action>::SharedPtr action_server;
	rclcpp::TimerBase::SharedPtr             compute_timer;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	std::shared_ptr<tf2_ros::Buffer>            tf_buffer;

	std::string base_link;
	float       max_joint_vel;
	double      update_rate;
	double      default_timeout_seconds;
	double      base_damping;
	double      max_damping;
	double      singularity_sigma_threshold;
	double      joint_centering_gain;
	bool        enable_singularity_logging;
	double      singularity_log_period_ms;
	double      linear_kp;
	double      angular_kp;
	double      max_linear_speed;
	double      min_linear_speed;
	double      min_speed_tolerance_scale;
	double      max_angular_speed;
	double      fine_approach_distance;
	double      fine_approach_angular_scale;
	double      fine_approach_posture_scale;
	double      position_tolerance;
	double      orientation_tolerance_rad;

	KDL::Chain                                arm_link_end_chain;
	KDL::Chain                                arm_link_gripper_chain;
	std::unique_ptr<KDL::ChainJntToJacSolver> arm_link_end_jacobian_solver;
	std::unique_ptr<KDL::ChainJntToJacSolver>
	    arm_link_gripper_jacobian_solver;
	KDL::Chain               *active_chain = nullptr;
	KDL::ChainJntToJacSolver *active_jacobian_solver = nullptr;
	KDL::JntArray                             current_joint_positions;
	Eigen::VectorXd                           joint_preferred_positions;
	Eigen::VectorXd                           joint_motion_weights;
	Eigen::VectorXd                           singularity_avoidance_gains;
	Eigen::VectorXd                           singularity_avoidance_thresholds;
	Eigen::VectorXd                           singularity_preferred_positions;
	bool                                      joints_initialized = false;
	bool                                      kinematics_ready = false;

	std::mutex                  state_mutex;
	std::shared_ptr<GoalHandle> active_goal;
	bool                        goal_reserved = false;
	rclcpp::Time                deadline;

	PoseIK(const rclcpp::NodeOptions &options) : Node("pose_ik", options) {
		declare_parameter<std::string>("base_link", "base_link");
		declare_parameter<float>("max_joint_vel", 0.5);
		declare_parameter<double>("update_rate", 10.0);
		declare_parameter<double>("default_timeout_seconds", 10.0);
		declare_parameter<double>("base_damping", 0.03);
		declare_parameter<double>("max_damping", 0.35);
		declare_parameter<double>("singularity_sigma_threshold", 0.12);
		declare_parameter<double>("joint_centering_gain", 0.35);
		declare_parameter<bool>("enable_singularity_logging", true);
		declare_parameter<double>("singularity_log_period_ms", 1000.0);
		declare_parameter<std::vector<double>>(
		    "joint_motion_weights", std::vector<double>{}
		);
		declare_parameter<std::vector<double>>(
		    "singularity_avoidance_gains", std::vector<double>{}
		);
		declare_parameter<std::vector<double>>(
		    "singularity_avoidance_thresholds", std::vector<double>{}
		);
		declare_parameter<std::vector<double>>(
		    "singularity_preferred_positions", std::vector<double>{}
		);
		declare_parameter<double>("linear_kp", 0.8);
		declare_parameter<double>("angular_kp", 2.0);
		declare_parameter<double>("max_linear_speed", 0.12);
		declare_parameter<double>("min_linear_speed", 0.015);
		declare_parameter<double>("min_speed_tolerance_scale", 1.5);
		declare_parameter<double>("max_angular_speed", 0.6);
		declare_parameter<double>("fine_approach_distance", 0.08);
		declare_parameter<double>("fine_approach_angular_scale", 0.2);
		declare_parameter<double>("fine_approach_posture_scale", 0.0);
		declare_parameter<double>("position_tolerance", 0.01);
		declare_parameter<double>("orientation_tolerance_rad", 0.15);

		get_parameter("base_link", base_link);
		get_parameter("max_joint_vel", max_joint_vel);
		get_parameter("update_rate", update_rate);
		get_parameter("default_timeout_seconds", default_timeout_seconds);
		if (!std::isfinite(default_timeout_seconds) ||
		    default_timeout_seconds <= 0.0) {
			RCLCPP_WARN(
			    get_logger(),
			    "default_timeout_seconds must be positive; using 10.0 seconds"
			);
			default_timeout_seconds = 10.0;
		}
		get_parameter("base_damping", base_damping);
		get_parameter("max_damping", max_damping);
		get_parameter(
		    "singularity_sigma_threshold", singularity_sigma_threshold
		);
		get_parameter("joint_centering_gain", joint_centering_gain);
		get_parameter("enable_singularity_logging", enable_singularity_logging);
		get_parameter("singularity_log_period_ms", singularity_log_period_ms);
		get_parameter("linear_kp", linear_kp);
		get_parameter("angular_kp", angular_kp);
		get_parameter("max_linear_speed", max_linear_speed);
		get_parameter("min_linear_speed", min_linear_speed);
		get_parameter("min_speed_tolerance_scale", min_speed_tolerance_scale);
		get_parameter("max_angular_speed", max_angular_speed);
		get_parameter("fine_approach_distance", fine_approach_distance);
		get_parameter(
		    "fine_approach_angular_scale", fine_approach_angular_scale
		);
		get_parameter(
		    "fine_approach_posture_scale", fine_approach_posture_scale
		);
		get_parameter("position_tolerance", position_tolerance);
		get_parameter("orientation_tolerance_rad", orientation_tolerance_rad);

		tf_buffer   = std::make_shared<tf2_ros::Buffer>(get_clock());
		tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

		joint_vel_pub = create_publisher<kalman_interfaces::msg::ArmValues>(
		    "target_vel", 10
		);

		joint_pos_sub = create_subscription<kalman_interfaces::msg::ArmValues>(
		    "current_pos",
		    10,
		    std::bind(&PoseIK::on_joint_positions, this, std::placeholders::_1)
		);

		using namespace std::placeholders;
		action_server = rclcpp_action::create_server<Action>(
		    this,
		    "pose_ik_navigate_to_pose",
		    std::bind(&PoseIK::handle_goal, this, _1, _2),
		    std::bind(&PoseIK::handle_cancel, this, _1),
		    std::bind(&PoseIK::handle_accepted, this, _1)
		);

		auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
		compute_timer     = create_wall_timer(
            timer_period, std::bind(&PoseIK::compute_joint_velocities, this)
        );

		rclcpp::QoS robot_description_qos(1);
		robot_description_qos.transient_local();
		robot_description_qos.reliable();

		robot_description_sub = create_subscription<std_msgs::msg::String>(
		    "/robot_description",
		    robot_description_qos,
		    std::bind(
		        &PoseIK::on_robot_description, this, std::placeholders::_1
		    )
		);
	}

	Eigen::VectorXd vector_from_parameter(
	    const std::string &name, size_t size, const Eigen::VectorXd &fallback
	) {
		auto values = get_parameter(name).as_double_array();
		if (values.empty()) {
			return fallback;
		}
		if (values.size() != size) {
			RCLCPP_WARN(
			    get_logger(),
			    "Parameter '%s' has %zu elements, expected %zu. Using "
			    "defaults.",
			    name.c_str(),
			    values.size(),
			    size
			);
			return fallback;
		}

		Eigen::VectorXd result(size);
		for (size_t i = 0; i < size; ++i) {
			result(i) = values[i];
		}
		return result;
	}

	std::string format_vector(const Eigen::VectorXd &values) const {
		std::ostringstream stream;
		stream << "[";
		for (Eigen::Index i = 0; i < values.size(); ++i) {
			if (i != 0) {
				stream << ", ";
			}
			stream << values(i);
		}
		stream << "]";
		return stream.str();
	}

	void on_robot_description(const std_msgs::msg::String::SharedPtr msg) {
		{
			std::lock_guard<std::mutex> lock(state_mutex);
			if (kinematics_ready) {
				return;
			}
		}

		urdf::Model model;
		if (!model.initString(msg->data)) {
			RCLCPP_ERROR(
			    get_logger(), "Failed to parse URDF from /robot_description."
			);
			return;
		}

		KDL::Tree kdl_tree;
		if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
			RCLCPP_ERROR(get_logger(), "Failed to create KDL tree from URDF.");
			return;
		}

		KDL::Chain end_chain;
		KDL::Chain gripper_chain;
		if (!kdl_tree.getChain(base_link, ARM_LINK_END, end_chain)) {
			RCLCPP_ERROR(
			    get_logger(),
			    "Failed to extract KDL chain from '%s' to '%s'.",
			    base_link.c_str(),
			    ARM_LINK_END
			);
			return;
		}
		if (!kdl_tree.getChain(base_link, ARM_LINK_GRIPPER, gripper_chain)) {
			RCLCPP_ERROR(
			    get_logger(),
			    "Failed to extract KDL chain from '%s' to '%s'.",
			    base_link.c_str(),
			    ARM_LINK_GRIPPER
			);
			return;
		}

		const size_t nj = end_chain.getNrOfJoints();
		if (nj == 0 || gripper_chain.getNrOfJoints() != nj) {
			RCLCPP_ERROR(
			    get_logger(),
			    "PoseIK chains must have the same non-zero movable joint count "
			    "('%s': %u, '%s': %u).",
			    ARM_LINK_END,
			    end_chain.getNrOfJoints(),
			    ARM_LINK_GRIPPER,
			    gripper_chain.getNrOfJoints()
			);
			return;
		}

		Eigen::VectorXd preferred_positions = Eigen::VectorXd::Zero(nj);
		Eigen::VectorXd motion_weights = Eigen::VectorXd::Ones(nj);
		Eigen::VectorXd avoidance_gains = Eigen::VectorXd::Zero(nj);
		Eigen::VectorXd avoidance_thresholds = Eigen::VectorXd::Zero(nj);
		Eigen::VectorXd singularity_positions = Eigen::VectorXd::Zero(nj);

		size_t joint_index = 0;
		for (const auto &segment : end_chain.segments) {
			const auto &joint = segment.getJoint();
			if (joint.getType() == KDL::Joint::None) {
				continue;
			}
			if (auto urdf_joint = model.getJoint(joint.getName())) {
				if (urdf_joint->limits) {
					preferred_positions(joint_index) =
					    0.5 *
					    (urdf_joint->limits->lower + urdf_joint->limits->upper);
				}
			}
			++joint_index;
		}

		if (nj > 3) {
			motion_weights(3) = 10.0;
		}
		if (nj > 4) {
			motion_weights(4) = 0.5;
		}
		if (nj > 5) {
			avoidance_gains(5)      = 1.2;
			avoidance_thresholds(5) = 0.35;
			singularity_positions(5) = 1.4;
		}

		motion_weights = vector_from_parameter(
		    "joint_motion_weights", nj, motion_weights
		);
		avoidance_gains = vector_from_parameter(
		    "singularity_avoidance_gains", nj, avoidance_gains
		);
		avoidance_thresholds = vector_from_parameter(
		    "singularity_avoidance_thresholds", nj, avoidance_thresholds
		);
		singularity_positions = vector_from_parameter(
		    "singularity_preferred_positions", nj, singularity_positions
		);

		std::lock_guard<std::mutex> lock(state_mutex);
		if (kinematics_ready) {
			return;
		}
		arm_link_end_chain = end_chain;
		arm_link_gripper_chain = gripper_chain;
		arm_link_end_jacobian_solver =
		    std::make_unique<KDL::ChainJntToJacSolver>(arm_link_end_chain);
		arm_link_gripper_jacobian_solver =
		    std::make_unique<KDL::ChainJntToJacSolver>(arm_link_gripper_chain);
		joint_preferred_positions = std::move(preferred_positions);
		joint_motion_weights = std::move(motion_weights);
		singularity_avoidance_gains = std::move(avoidance_gains);
		singularity_avoidance_thresholds = std::move(avoidance_thresholds);
		singularity_preferred_positions = std::move(singularity_positions);
		current_joint_positions.resize(nj);
		kinematics_ready = true;

		RCLCPP_INFO(
		    get_logger(),
		    "PoseIK initialized both tip chains with %zu joints. "
		    "motion_weights=%s avoidance_gains=%s",
		    nj,
		    format_vector(joint_motion_weights).c_str(),
		    format_vector(singularity_avoidance_gains).c_str()
		);
	}

	void
	on_joint_positions(const kalman_interfaces::msg::ArmValues::SharedPtr msg) {
		std::lock_guard<std::mutex> lock(state_mutex);
		if (!kinematics_ready) {
			return;
		}
		if (msg->joints.size() < current_joint_positions.rows()) {
			return;
		}
		for (size_t i = 0; i < current_joint_positions.rows(); ++i) {
			current_joint_positions(i) = msg->joints[i];
		}
		joints_initialized = true;
	}

	rclcpp_action::GoalResponse handle_goal(
	    const rclcpp_action::GoalUUID &uuid,
	    const std::shared_ptr<const Action::Goal> goal
	) {
		(void)uuid;
		std::lock_guard<std::mutex> lock(state_mutex);
		if (!kinematics_ready) {
			RCLCPP_WARN(get_logger(), "Rejecting PoseIK goal: kinematics not ready");
			return rclcpp_action::GoalResponse::REJECT;
		}
		if (goal_reserved || active_goal) {
			RCLCPP_WARN(get_logger(), "Rejecting PoseIK goal: server is busy");
			return rclcpp_action::GoalResponse::REJECT;
		}
		if (goal->end_effector_link != ARM_LINK_END &&
		    goal->end_effector_link != ARM_LINK_GRIPPER) {
			RCLCPP_WARN(
			    get_logger(),
			    "Rejecting PoseIK goal: unsupported end-effector link '%s'",
			    goal->end_effector_link.c_str()
			);
			return rclcpp_action::GoalResponse::REJECT;
		}
		if (goal->base_frame.empty()) {
			RCLCPP_WARN(get_logger(), "Rejecting PoseIK goal: base frame is empty");
			return rclcpp_action::GoalResponse::REJECT;
		}

		goal_reserved = true;
		return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
	}

	rclcpp_action::CancelResponse
	handle_cancel(const std::shared_ptr<GoalHandle> goal_handle) {
		(void)goal_handle;
		return rclcpp_action::CancelResponse::ACCEPT;
	}

	void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
		std::lock_guard<std::mutex> lock(state_mutex);
		const auto &goal = *goal_handle->get_goal();
		if (goal.end_effector_link == ARM_LINK_END) {
			active_chain = &arm_link_end_chain;
			active_jacobian_solver = arm_link_end_jacobian_solver.get();
		} else {
			active_chain = &arm_link_gripper_chain;
			active_jacobian_solver = arm_link_gripper_jacobian_solver.get();
		}

		const double timeout =
		    std::isfinite(goal.timeout_seconds) && goal.timeout_seconds > 0.0
		        ? goal.timeout_seconds
		        : default_timeout_seconds;
		deadline = now() + rclcpp::Duration::from_seconds(timeout);
		active_goal = goal_handle;
		goal_reserved = false;
	}

	Eigen::VectorXd current_joint_positions_eigen() const {
		Eigen::VectorXd q(current_joint_positions.rows());
		for (size_t i = 0; i < current_joint_positions.rows(); ++i) {
			q(i) = current_joint_positions(i);
		}
		return q;
	}

	double compute_adaptive_damping(double min_sigma) const {
		if (min_sigma >= singularity_sigma_threshold) {
			return base_damping;
		}
		if (singularity_sigma_threshold <= 0.0) {
			return max_damping;
		}
		const double ratio = 1.0 - (min_sigma / singularity_sigma_threshold);
		return base_damping + (max_damping - base_damping) * ratio * ratio;
	}

	double compute_min_singular_value(const Eigen::MatrixXd &jacobian) const {
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(
		    jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV
		);
		if (svd.singularValues().size() == 0) {
			return 0.0;
		}
		return svd.singularValues().minCoeff();
	}

	Eigen::VectorXd compute_posture_bias(const Eigen::VectorXd &q) const {
		Eigen::VectorXd bias =
		    joint_centering_gain * (joint_preferred_positions - q);
		for (Eigen::Index i = 0; i < q.size(); ++i) {
			const double gain      = singularity_avoidance_gains(i);
			const double threshold = singularity_avoidance_thresholds(i);
			if (gain <= 0.0 || threshold <= 0.0) {
				continue;
			}
			const double joint_angle = q(i);
			if (std::abs(joint_angle) < threshold) {
				const double preferred = singularity_preferred_positions(i);
				const double direction = joint_angle >= 0.0 ? 1.0 : -1.0;
				const double signed_preferred =
				    preferred == 0.0 ? 0.0 : direction * std::abs(preferred);
				const double ratio = 1.0 - (std::abs(joint_angle) / threshold);
				bias(i) += gain * ratio * (signed_preferred - joint_angle);
			}
		}
		return bias;
	}

	bool solve_joint_velocities(
	    const geometry_msgs::msg::Twist &base_twist,
	    double                           position_error,
	    bool                             position_only,
	    Eigen::VectorXd                 &joint_velocities,
	    double                          &min_sigma_out,
	    double                          &damping_out,
	    Eigen::Matrix<double, 6, 1>     &achieved_twist_out
	) {
		if (!active_chain || !active_jacobian_solver) {
			return false;
		}
		KDL::Jacobian kdl_jacobian(active_chain->getNrOfJoints());
		const int     jacobian_result = active_jacobian_solver->JntToJac(
		    current_joint_positions, kdl_jacobian
		);
		if (jacobian_result < 0) {
			return false;
		}

		const Eigen::MatrixXd jacobian = kdl_jacobian.data;
		Eigen::MatrixXd       weight_inv =
		    joint_motion_weights.cwiseInverse().asDiagonal();

		Eigen::MatrixXd active_jacobian;
		Eigen::VectorXd twist_vector;
		if (position_only) {
			active_jacobian = jacobian.topRows(3);
			twist_vector.resize(3);
			twist_vector << base_twist.linear.x, base_twist.linear.y,
			    base_twist.linear.z;
		} else {
			active_jacobian = jacobian;
			twist_vector.resize(6);
			twist_vector << base_twist.linear.x, base_twist.linear.y,
			    base_twist.linear.z, base_twist.angular.x, base_twist.angular.y,
			    base_twist.angular.z;
		}

		const double min_sigma = compute_min_singular_value(active_jacobian);
		const double damping   = compute_adaptive_damping(min_sigma);
		const Eigen::MatrixXd task_metric =
		    active_jacobian * weight_inv * active_jacobian.transpose() +
		    (damping * damping) *
		        Eigen::MatrixXd::Identity(
		            active_jacobian.rows(), active_jacobian.rows()
		        );

		const Eigen::MatrixXd weighted_pseudoinverse =
		    weight_inv * active_jacobian.transpose() *
		    task_metric.ldlt().solve(
		        Eigen::MatrixXd::Identity(
		            active_jacobian.rows(), active_jacobian.rows()
		        )
		    );

		const Eigen::VectorXd task_velocity =
		    weighted_pseudoinverse * twist_vector;
		const Eigen::VectorXd q            = current_joint_positions_eigen();
		Eigen::VectorXd       posture_bias = compute_posture_bias(q);
		if (position_error < fine_approach_distance) {
			posture_bias *= fine_approach_posture_scale;
		}
		const Eigen::MatrixXd nullspace_projector =
		    Eigen::MatrixXd::Identity(q.size(), q.size()) -
		    weighted_pseudoinverse * active_jacobian;

		joint_velocities   = task_velocity + nullspace_projector * posture_bias;
		min_sigma_out      = min_sigma;
		damping_out        = damping;
		achieved_twist_out = jacobian * joint_velocities;

		if (enable_singularity_logging) {
			RCLCPP_INFO_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    static_cast<int64_t>(singularity_log_period_ms),
			    "PoseIK diagnostics: sigma_min=%.4f damping=%.4f mode=%s "
			    "qdot=%s",
			    min_sigma,
			    damping,
			    position_only ? "position_only" : "full_pose",
			    format_vector(joint_velocities).c_str()
			);
		}
		return true;
	}

	geometry_msgs::msg::Pose transform_pose_to_base_frame(
	    const geometry_msgs::msg::Pose &pose, const std::string &source_frame
	) {
		if (source_frame == base_link) {
			return pose;
		}

		const auto transform = tf_buffer->lookupTransform(
		    base_link,
		    source_frame,
		    rclcpp::Time(0, 0, get_clock()->get_clock_type()),
		    rclcpp::Duration::from_seconds(0.1)
		);
		geometry_msgs::msg::Pose base_pose;
		tf2::doTransform(pose, base_pose, transform);
		return base_pose;
	}

	geometry_msgs::msg::Pose
	current_ee_pose(const std::string &end_effector_link) {
		auto transform = tf_buffer->lookupTransform(
		    base_link,
		    end_effector_link,
		    rclcpp::Time(0, 0, get_clock()->get_clock_type()),
		    rclcpp::Duration::from_seconds(0.1)
		);
		geometry_msgs::msg::Pose pose;
		pose.position.x  = transform.transform.translation.x;
		pose.position.y  = transform.transform.translation.y;
		pose.position.z  = transform.transform.translation.z;
		pose.orientation = transform.transform.rotation;
		return pose;
	}

	double quaternion_angle_to_target(
	    const geometry_msgs::msg::Quaternion &current_msg,
	    const geometry_msgs::msg::Quaternion &target_msg
	) const {
		tf2::Quaternion q_current, q_target;
		tf2::fromMsg(current_msg, q_current);
		tf2::fromMsg(target_msg, q_target);
		q_current.normalize();
		q_target.normalize();
		tf2::Quaternion q_error = q_current.inverse() * q_target;
		q_error.normalize();
		if (q_error.getW() < 0.0) {
			q_error = tf2::Quaternion(
			    -q_error.getX(),
			    -q_error.getY(),
			    -q_error.getZ(),
			    -q_error.getW()
			);
		}
		const double w =
		    std::clamp(static_cast<double>(q_error.getW()), -1.0, 1.0);
		return 2.0 * std::acos(w);
	}

	geometry_msgs::msg::Twist pose_error_to_twist(
	    const geometry_msgs::msg::Pose &current,
	    const geometry_msgs::msg::Pose &target,
	    double                          position_error
	) const {
		geometry_msgs::msg::Twist twist;
		tf2::Vector3              linear_error(
            target.position.x - current.position.x,
            target.position.y - current.position.y,
            target.position.z - current.position.z
        );
		tf2::Vector3 linear_cmd   = linear_error * linear_kp;
		const double linear_speed = linear_cmd.length();
		const double min_speed_disable_distance =
		    position_tolerance * min_speed_tolerance_scale;
		if (linear_speed > max_linear_speed) {
			linear_cmd *= max_linear_speed / linear_speed;
		} else if (linear_speed > 1e-9 &&
		           position_error > min_speed_disable_distance &&
		           linear_speed < min_linear_speed) {
			linear_cmd *= min_linear_speed / linear_speed;
		}
		twist.linear.x = linear_cmd.x();
		twist.linear.y = linear_cmd.y();
		twist.linear.z = linear_cmd.z();

		tf2::Quaternion q_current, q_target;
		tf2::fromMsg(current.orientation, q_current);
		tf2::fromMsg(target.orientation, q_target);
		q_current.normalize();
		q_target.normalize();
		// KDL Jacobian angular rows are expressed in base_link, so use the
		// spatial rotation error rather than the body-frame rotation error.
		tf2::Quaternion q_error = q_target * q_current.inverse();
		q_error.normalize();
		if (q_error.getW() < 0.0) {
			q_error = tf2::Quaternion(
			    -q_error.getX(),
			    -q_error.getY(),
			    -q_error.getZ(),
			    -q_error.getW()
			);
		}
		const double w =
		    std::clamp(static_cast<double>(q_error.getW()), -1.0, 1.0);
		const double angle = 2.0 * std::acos(w);
		const double s     = std::sqrt(std::max(1e-16, 1.0 - w * w));
		tf2::Vector3 axis(1.0, 0.0, 0.0);
		if (s > 1e-8) {
			axis = tf2::Vector3(
			    q_error.getX() / s, q_error.getY() / s, q_error.getZ() / s
			);
		}
		tf2::Vector3 angular_cmd   = axis * angle * angular_kp;
		const double angular_speed = angular_cmd.length();
		if (angular_speed > max_angular_speed) {
			angular_cmd *= max_angular_speed / angular_speed;
		}
		twist.angular.x = angular_cmd.x();
		twist.angular.y = angular_cmd.y();
		twist.angular.z = angular_cmd.z();

		if (position_error < fine_approach_distance) {
			twist.angular.x *= fine_approach_angular_scale;
			twist.angular.y *= fine_approach_angular_scale;
			twist.angular.z *= fine_approach_angular_scale;
		}
		return twist;
	}

	void publish_zero_velocity() {
		auto vel_msg         = kalman_interfaces::msg::ArmValues();
		vel_msg.header.stamp = now();
		vel_msg.joints.fill(0.0);
		vel_msg.jaw = 0.0;
		joint_vel_pub->publish(vel_msg);
	}

	void clear_active_goal() {
		active_goal.reset();
		active_chain = nullptr;
		active_jacobian_solver = nullptr;
		goal_reserved = false;
	}

	void compute_joint_velocities() {
		std::lock_guard<std::mutex> lock(state_mutex);
		if (!active_goal) {
			return;
		}

		if (active_goal->is_canceling()) {
			publish_zero_velocity();
			auto result = std::make_shared<Action::Result>();
			result->result = false;
			result->message = "Pose IK navigation canceled";
			active_goal->canceled(result);
			clear_active_goal();
			return;
		}
		if (!active_goal->is_active()) {
			publish_zero_velocity();
			clear_active_goal();
			return;
		}
		if (now() >= deadline) {
			publish_zero_velocity();
			auto result = std::make_shared<Action::Result>();
			result->result = false;
			result->message = "Pose IK navigation timed out";
			active_goal->abort(result);
			clear_active_goal();
			return;
		}
		if (!kinematics_ready || !joints_initialized) {
			return;
		}

		const auto goal = active_goal->get_goal();
		try {
			const geometry_msgs::msg::Pose target_pose =
			    transform_pose_to_base_frame(goal->pose, goal->base_frame);
			const geometry_msgs::msg::Pose current_pose =
			    current_ee_pose(goal->end_effector_link);

			const double dx = target_pose.position.x - current_pose.position.x;
			const double dy = target_pose.position.y - current_pose.position.y;
			const double dz = target_pose.position.z - current_pose.position.z;
			const double position_error =
			    std::sqrt(dx * dx + dy * dy + dz * dz);
			const double orientation_error = quaternion_angle_to_target(
			    current_pose.orientation, target_pose.orientation
			);

			auto feedback = std::make_shared<Action::Feedback>();
			feedback->position_error = position_error;
			feedback->orientation_error = orientation_error;
			active_goal->publish_feedback(feedback);

			if (position_error <= position_tolerance &&
			    orientation_error <= orientation_tolerance_rad) {
				publish_zero_velocity();
				auto result = std::make_shared<Action::Result>();
				result->result = true;
				result->message = "Target pose reached";
				active_goal->succeed(result);
				clear_active_goal();
				return;
			}

			const geometry_msgs::msg::Twist base_twist =
			    pose_error_to_twist(current_pose, target_pose, position_error);
			const bool position_only =
			    orientation_error <= orientation_tolerance_rad &&
			    position_error < fine_approach_distance;

			Eigen::VectorXd             joint_velocities;
			double                      min_sigma = 0.0;
			double                      damping   = 0.0;
			Eigen::Matrix<double, 6, 1> achieved_twist =
			    Eigen::Matrix<double, 6, 1>::Zero();
			if (!solve_joint_velocities(
			        base_twist,
			        position_error,
			        position_only,
			        joint_velocities,
			        min_sigma,
			        damping,
			        achieved_twist
			    )) {
				return;
			}

			const double max_computed_vel =
			    joint_velocities.cwiseAbs().maxCoeff();
			if (max_computed_vel > max_joint_vel) {
				joint_velocities *= max_joint_vel / max_computed_vel;
			}

			auto vel_msg         = kalman_interfaces::msg::ArmValues();
			vel_msg.header.stamp = now();
			for (Eigen::Index i = 0; i < joint_velocities.size(); ++i) {
				vel_msg.joints[i] = joint_velocities(i);
			}
			joint_vel_pub->publish(vel_msg);

			const double achieved_linear_speed = std::sqrt(
			    achieved_twist(0) * achieved_twist(0) +
			    achieved_twist(1) * achieved_twist(1) +
			    achieved_twist(2) * achieved_twist(2)
			);
			const double commanded_linear_speed = std::sqrt(
			    base_twist.linear.x * base_twist.linear.x +
			    base_twist.linear.y * base_twist.linear.y +
			    base_twist.linear.z * base_twist.linear.z
			);
			const char *gate_state =
			    position_error > position_tolerance
			        ? (orientation_error > orientation_tolerance_rad
			               ? "position+orientation"
			               : "position")
			        : "orientation";
#if 0
			RCLCPP_INFO_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    1000,
			    "PoseIK target=(%.3f %.3f %.3f) current=(%.3f %.3f %.3f) "
			    "pos_err=(%.3f %.3f %.3f) pos_norm=%.4f ang_err=%.4f gate=%s "
			    "mode=%s cmd_v=(%.3f %.3f %.3f) |cmd_v|=%.4f "
			    "cmd_w=(%.3f %.3f %.3f) sigma_min=%.4f damping=%.4f "
			    "q=(%.3f %.3f %.3f %.3f %.3f %.3f) "
			    "qdot=(%.5f %.5f %.5f %.5f %.5f %.5f) "
			    "achieved_v=(%.3f %.3f %.3f) |achieved_v|=%.4f",
			    target_pose.position.x,
			    target_pose.position.y,
			    target_pose.position.z,
			    current_pose.position.x,
			    current_pose.position.y,
			    current_pose.position.z,
			    dx,
			    dy,
			    dz,
			    position_error,
			    orientation_error,
			    gate_state,
			    position_only ? "position_only" : "full_pose",
			    base_twist.linear.x,
			    base_twist.linear.y,
			    base_twist.linear.z,
			    commanded_linear_speed,
			    base_twist.angular.x,
			    base_twist.angular.y,
			    base_twist.angular.z,
			    min_sigma,
			    damping,
			    current_joint_positions(0),
			    current_joint_positions(1),
			    current_joint_positions(2),
			    current_joint_positions(3),
			    current_joint_positions(4),
			    current_joint_positions(5),
			    joint_velocities(0),
			    joint_velocities(1),
			    joint_velocities(2),
			    joint_velocities(3),
			    joint_velocities(4),
			    joint_velocities(5),
			    achieved_twist(0),
			    achieved_twist(1),
			    achieved_twist(2),
			    achieved_linear_speed
			);
#endif
		} catch (const tf2::TransformException &ex) {
			RCLCPP_WARN_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    1000,
			    "PoseIK TF problem for '%s' in '%s': %s",
			    goal->end_effector_link.c_str(),
			    goal->base_frame.c_str(),
			    ex.what()
			);
		}
	}
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PoseIK)
