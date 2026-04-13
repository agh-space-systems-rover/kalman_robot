#include <geometry_msgs/msg/pose_stamped.hpp>
#include <kalman_interfaces/msg/arm_values.hpp>
#include <rclcpp/rclcpp.hpp>
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
#include <limits>
#include <mutex>
#include <sstream>
#include <vector>

namespace kalman_arm2 {

class PoseIK : public rclcpp::Node {
  public:
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub;
	rclcpp::Subscription<kalman_interfaces::msg::ArmValues>::SharedPtr
	    joint_pos_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
	    robot_description_sub;
	rclcpp::Publisher<kalman_interfaces::msg::ArmValues>::SharedPtr
	                             joint_vel_pub;
	rclcpp::TimerBase::SharedPtr compute_timer;

	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	std::shared_ptr<tf2_ros::Buffer>            tf_buffer;

	std::string base_link;
	std::string end_effector_link;
	float       max_joint_vel;
	double      update_rate;
	double      control_timeout;
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
	double      min_speed_activation_distance;
	double      max_angular_speed;
	double      fine_approach_distance;
	double      fine_approach_angular_scale;
	double      position_tolerance;
	double      orientation_tolerance_rad;

	KDL::Chain                                arm_chain;
	std::unique_ptr<KDL::ChainJntToJacSolver> jacobian_solver;
	KDL::JntArray                             current_joint_positions;
	Eigen::VectorXd                           joint_preferred_positions;
	Eigen::VectorXd                           joint_motion_weights;
	Eigen::VectorXd                           singularity_avoidance_gains;
	Eigen::VectorXd                           singularity_avoidance_thresholds;
	Eigen::VectorXd                           singularity_preferred_positions;
	bool                                      joints_initialized;
	bool                                      kinematics_ready;
	bool                                      was_active;

	geometry_msgs::msg::PoseStamped::SharedPtr latest_pose_goal;
	std::mutex                                 pose_mutex;
	rclcpp::Time                               last_goal_time;

	PoseIK(const rclcpp::NodeOptions &options)
	    : Node("pose_ik", options), joints_initialized(false),
	      kinematics_ready(false), was_active(false), last_goal_time(now()) {
		declare_parameter<std::string>("base_link", "base_link");
		declare_parameter<std::string>("end_effector_link", "arm_link_end");
		declare_parameter<float>("max_joint_vel", 0.5);
		declare_parameter<double>("update_rate", 10.0);
		declare_parameter<double>("control_timeout", 0.5);
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
		declare_parameter<double>("min_speed_activation_distance", 0.03);
		declare_parameter<double>("max_angular_speed", 0.6);
		declare_parameter<double>("fine_approach_distance", 0.08);
		declare_parameter<double>("fine_approach_angular_scale", 0.2);
		declare_parameter<double>("position_tolerance", 0.01);
		declare_parameter<double>("orientation_tolerance_rad", 0.15);

		get_parameter("base_link", base_link);
		get_parameter("end_effector_link", end_effector_link);
		get_parameter("max_joint_vel", max_joint_vel);
		get_parameter("update_rate", update_rate);
		get_parameter("control_timeout", control_timeout);
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
		get_parameter(
		    "min_speed_activation_distance", min_speed_activation_distance
		);
		get_parameter("max_angular_speed", max_angular_speed);
		get_parameter("fine_approach_distance", fine_approach_distance);
		get_parameter(
		    "fine_approach_angular_scale", fine_approach_angular_scale
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

		pose_sub = create_subscription<geometry_msgs::msg::PoseStamped>(
		    "target_pose",
		    10,
		    std::bind(&PoseIK::on_target_pose, this, std::placeholders::_1)
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
		if (kinematics_ready) {
			return;
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

		if (!kdl_tree.getChain(base_link, end_effector_link, arm_chain)) {
			RCLCPP_ERROR(
			    get_logger(),
			    "Failed to extract KDL chain from '%s' to '%s'.",
			    base_link.c_str(),
			    end_effector_link.c_str()
			);
			return;
		}

		size_t nj       = arm_chain.getNrOfJoints();
		jacobian_solver = std::make_unique<KDL::ChainJntToJacSolver>(arm_chain);
		joint_preferred_positions        = Eigen::VectorXd::Zero(nj);
		joint_motion_weights             = Eigen::VectorXd::Ones(nj);
		singularity_avoidance_gains      = Eigen::VectorXd::Zero(nj);
		singularity_avoidance_thresholds = Eigen::VectorXd::Zero(nj);
		singularity_preferred_positions  = Eigen::VectorXd::Zero(nj);

		size_t joint_index = 0;
		for (const auto &segment : arm_chain.segments) {
			const auto &joint = segment.getJoint();
			if (joint.getType() == KDL::Joint::None) {
				continue;
			}
			if (auto urdf_joint = model.getJoint(joint.getName())) {
				if (urdf_joint->limits) {
					joint_preferred_positions(joint_index) =
					    0.5 *
					    (urdf_joint->limits->lower + urdf_joint->limits->upper);
				}
			}
			++joint_index;
		}

		if (nj > 3) {
			joint_motion_weights(3) = 10.0;
		}
		if (nj > 4) {
			joint_motion_weights(4) = 0.5;
		}
		if (nj > 5) {
			singularity_avoidance_gains(5)      = 1.2;
			singularity_avoidance_thresholds(5) = 0.35;
			singularity_preferred_positions(5)  = 1.4;
		}

		joint_motion_weights = vector_from_parameter(
		    "joint_motion_weights", nj, joint_motion_weights
		);
		singularity_avoidance_gains = vector_from_parameter(
		    "singularity_avoidance_gains", nj, singularity_avoidance_gains
		);
		singularity_avoidance_thresholds = vector_from_parameter(
		    "singularity_avoidance_thresholds",
		    nj,
		    singularity_avoidance_thresholds
		);
		singularity_preferred_positions = vector_from_parameter(
		    "singularity_preferred_positions",
		    nj,
		    singularity_preferred_positions
		);

		current_joint_positions.resize(arm_chain.getNrOfJoints());
		kinematics_ready = true;

		RCLCPP_INFO(
		    get_logger(),
		    "PoseIK initialized with %d joints. motion_weights=%s "
		    "avoidance_gains=%s",
		    arm_chain.getNrOfJoints(),
		    format_vector(joint_motion_weights).c_str(),
		    format_vector(singularity_avoidance_gains).c_str()
		);
	}

	void
	on_joint_positions(const kalman_interfaces::msg::ArmValues::SharedPtr msg) {
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

	void on_target_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
		std::lock_guard<std::mutex> lock(pose_mutex);
		latest_pose_goal = msg;
		last_goal_time   = now();
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
	    Eigen::VectorXd                 &joint_velocities,
	    double                          &min_sigma_out,
	    double                          &damping_out,
	    Eigen::Matrix<double, 6, 1>     &achieved_twist_out
	) {
		KDL::Jacobian kdl_jacobian(current_joint_positions.rows());
		const int     jacobian_result =
		    jacobian_solver->JntToJac(current_joint_positions, kdl_jacobian);
		if (jacobian_result < 0) {
			return false;
		}

		const Eigen::MatrixXd jacobian = kdl_jacobian.data;
		Eigen::MatrixXd       weight_inv =
		    joint_motion_weights.cwiseInverse().asDiagonal();

		Eigen::Matrix<double, 6, 1> twist_vector;
		twist_vector << base_twist.linear.x, base_twist.linear.y,
		    base_twist.linear.z, base_twist.angular.x, base_twist.angular.y,
		    base_twist.angular.z;

		const double min_sigma = compute_min_singular_value(jacobian);
		const double damping   = compute_adaptive_damping(min_sigma);
		const Eigen::Matrix<double, 6, 6> task_metric =
		    jacobian * weight_inv * jacobian.transpose() +
		    (damping * damping) * Eigen::Matrix<double, 6, 6>::Identity();

		const Eigen::MatrixXd weighted_pseudoinverse =
		    weight_inv * jacobian.transpose() *
		    task_metric.ldlt().solve(Eigen::Matrix<double, 6, 6>::Identity());

		const Eigen::VectorXd task_velocity =
		    weighted_pseudoinverse * twist_vector;
		const Eigen::VectorXd q            = current_joint_positions_eigen();
		const Eigen::VectorXd posture_bias = compute_posture_bias(q);
		const Eigen::MatrixXd nullspace_projector =
		    Eigen::MatrixXd::Identity(q.size(), q.size()) -
		    weighted_pseudoinverse * jacobian;

		joint_velocities   = task_velocity + nullspace_projector * posture_bias;
		min_sigma_out      = min_sigma;
		damping_out        = damping;
		achieved_twist_out = jacobian * joint_velocities;

		if (enable_singularity_logging) {
			RCLCPP_INFO_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    static_cast<int64_t>(singularity_log_period_ms),
			    "PoseIK diagnostics: sigma_min=%.4f damping=%.4f qdot=%s",
			    min_sigma,
			    damping,
			    format_vector(joint_velocities).c_str()
			);
		}
		return true;
	}

	geometry_msgs::msg::Pose transform_pose_to_base_frame(
	    const geometry_msgs::msg::PoseStamped::SharedPtr &msg
	) {
		geometry_msgs::msg::Pose base_pose = msg->pose;
		if (!msg->header.frame_id.empty() &&
		    msg->header.frame_id != base_link) {
			try {
				geometry_msgs::msg::TransformStamped transform =
				    tf_buffer->lookupTransform(
				        base_link,
				        msg->header.frame_id,
				        msg->header.stamp,
				        rclcpp::Duration::from_seconds(0.1)
				    );
				tf2::doTransform(msg->pose, base_pose, transform);
			} catch (const tf2::TransformException &ex) {
				RCLCPP_WARN_THROTTLE(
				    get_logger(),
				    *get_clock(),
				    1000,
				    "Could not transform pose from '%s' to '%s': %s",
				    msg->header.frame_id.c_str(),
				    base_link.c_str(),
				    ex.what()
				);
			}
		}
		return base_pose;
	}

	geometry_msgs::msg::Pose current_ee_pose() {
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
		if (linear_speed > max_linear_speed) {
			linear_cmd *= max_linear_speed / linear_speed;
		} else if (linear_speed > 1e-9 &&
		           linear_error.length() > min_speed_activation_distance &&
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

	void compute_joint_velocities() {
		if (!kinematics_ready || !joints_initialized) {
			return;
		}

		geometry_msgs::msg::PoseStamped::SharedPtr goal_msg;
		{
			std::lock_guard<std::mutex> lock(pose_mutex);
			if (!latest_pose_goal ||
			    (now() - last_goal_time).seconds() > control_timeout) {
				if (was_active) {
					publish_zero_velocity();
					was_active = false;
				}
				return;
			}
			goal_msg = latest_pose_goal;
		}
		was_active = true;

		try {
			const geometry_msgs::msg::Pose target_pose =
			    transform_pose_to_base_frame(goal_msg);
			const geometry_msgs::msg::Pose current_pose = current_ee_pose();

			const double dx = target_pose.position.x - current_pose.position.x;
			const double dy = target_pose.position.y - current_pose.position.y;
			const double dz = target_pose.position.z - current_pose.position.z;
			const double position_error =
			    std::sqrt(dx * dx + dy * dy + dz * dz);
			const double orientation_error = quaternion_angle_to_target(
			    current_pose.orientation, target_pose.orientation
			);

			if (position_error <= position_tolerance &&
			    orientation_error <= orientation_tolerance_rad) {
				publish_zero_velocity();
				was_active = false;
				return;
			}

			const geometry_msgs::msg::Twist base_twist =
			    pose_error_to_twist(current_pose, target_pose, position_error);

			Eigen::VectorXd             joint_velocities;
			double                      min_sigma = 0.0;
			double                      damping   = 0.0;
			Eigen::Matrix<double, 6, 1> achieved_twist =
			    Eigen::Matrix<double, 6, 1>::Zero();
			if (!solve_joint_velocities(
			        base_twist,
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
                "cmd_v=(%.3f %.3f %.3f) |cmd_v|=%.4f "
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
			    "PoseIK TF problem: %s",
			    ex.what()
			);
		}
	}
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::PoseIK)
