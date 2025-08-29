#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kalman_interfaces/msg/arm_values.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <std_msgs/msg/string.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <mutex>
#include <vector>

namespace kalman_arm2 {

class TwistIK : public rclcpp::Node {
  public:
	// ROS interfaces
	rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub;
	rclcpp::Subscription<kalman_interfaces::msg::ArmValues>::SharedPtr
	    joint_pos_sub;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
	    robot_description_sub;
	rclcpp::Publisher<kalman_interfaces::msg::ArmValues>::SharedPtr
	                             joint_vel_pub;
	rclcpp::TimerBase::SharedPtr compute_timer;

	// TF2
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	std::shared_ptr<tf2_ros::Buffer>            tf_buffer;

	// Parameters
	std::string base_link;
	std::string end_effector_link;
	float       max_joint_vel;
	double      update_rate;
	double      control_timeout;

	// Kinematics
	KDL::Chain                                      arm_chain;
	std::unique_ptr<KDL::ChainIkSolverVel_pinv_nso> ik_solver;
	std::unique_ptr<KDL::ChainJntToJacSolver>       jac_solver;
	KDL::JntArray                                   current_joint_positions;
	KDL::JntArray q_nom; // preferred posture (mid-range)
	bool          joints_initialized;
	bool          kinematics_ready;

	// Joint limits (from URDF)
	std::vector<std::string> joint_names_;
	std::vector<double>      q_min_, q_max_, q_mid_;

	// Latest twist message
	geometry_msgs::msg::TwistStamped::SharedPtr latest_twist;
	std::mutex                                  twist_mutex;
	rclcpp::Time                                last_twist_time;

	TwistIK(const rclcpp::NodeOptions &options)
	    : Node("twist_ik", options), joints_initialized(false),
	      kinematics_ready(false), last_twist_time(now()) {

		// Parameters
		this->declare_parameter<std::string>("base_link", "base_link");
		this->declare_parameter<std::string>(
		    "end_effector_link", "arm_link_end"
		);
		this->declare_parameter<float>("max_joint_vel", 0.5);
		this->declare_parameter<double>("update_rate", 10.0);
		this->declare_parameter<double>("control_timeout", 0.5);

		this->get_parameter("base_link", base_link);
		this->get_parameter("end_effector_link", end_effector_link);
		this->get_parameter("max_joint_vel", max_joint_vel);
		this->get_parameter("update_rate", update_rate);
		this->get_parameter("control_timeout", control_timeout);

		// TF2
		tf_buffer   = std::make_shared<tf2_ros::Buffer>(get_clock());
		tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

		// Pub/Sub
		joint_vel_pub = create_publisher<kalman_interfaces::msg::ArmValues>(
		    "target_vel", 10
		);
		joint_pos_sub = create_subscription<kalman_interfaces::msg::ArmValues>(
		    "current_pos",
		    10,
		    std::bind(&TwistIK::on_joint_positions, this, std::placeholders::_1)
		);
		twist_sub = create_subscription<geometry_msgs::msg::TwistStamped>(
		    "target_twist",
		    10,
		    std::bind(&TwistIK::on_target_twist, this, std::placeholders::_1)
		);

		// Timer
		auto timer_period = std::chrono::duration<double>(1.0 / update_rate);
		compute_timer     = create_wall_timer(
            timer_period, std::bind(&TwistIK::compute_joint_velocities, this)
        );

		// robot_description (latched)
		rclcpp::QoS robot_description_qos(1);
		robot_description_qos.transient_local();
		robot_description_qos.reliable();
		robot_description_sub = create_subscription<std_msgs::msg::String>(
		    "/robot_description",
		    robot_description_qos,
		    std::bind(
		        &TwistIK::on_robot_description, this, std::placeholders::_1
		    )
		);

		RCLCPP_INFO(
		    get_logger(),
		    "TwistIK node waiting for robot description on "
		    "/robot_description..."
		);
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

		// Jacobian solver
		jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(arm_chain);

		// IK (NSO) solver
		ik_solver = std::make_unique<KDL::ChainIkSolverVel_pinv_nso>(
		    arm_chain, 1e-6, 150, 0.2
		);

		// Preferred posture (filled with mid-range after limits parsed)
		q_nom = KDL::JntArray(arm_chain.getNrOfJoints());

		// Parse joint limits from URDF for joints in the chain
		joint_names_.clear();
		q_min_.clear();
		q_max_.clear();
		q_mid_.clear();
		for (unsigned seg = 0; seg < arm_chain.getNrOfSegments(); ++seg) {
			const auto &joint = arm_chain.getSegment(seg).getJoint();
			if (joint.getType() == KDL::Joint::None) {
				continue;
			}
			joint_names_.push_back(joint.getName());

			auto urdf_joint = model.getJoint(joint.getName());
			if (urdf_joint && urdf_joint->limits) {
				q_min_.push_back(urdf_joint->limits->lower);
				q_max_.push_back(urdf_joint->limits->upper);
			} else {
				// If no limits, use very wide bounds
				q_min_.push_back(-1e9);
				q_max_.push_back(1e9);
			}
			q_mid_.push_back(0.5 * (q_min_.back() + q_max_.back()));
		}

		// Fill q_nom with mid-ranges and configure NSO
		for (unsigned i = 0; i < q_nom.rows(); ++i) {
			q_nom(i) = q_mid_[i];
		}
		ik_solver->setOptPos(q_nom);
		ik_solver->setAlpha(0.8); // strength of null-space pull

		current_joint_positions.resize(arm_chain.getNrOfJoints());

		kinematics_ready = true;
		RCLCPP_INFO(
		    get_logger(),
		    "Kinematic chain initialized with %d joints.",
		    arm_chain.getNrOfJoints()
		);
	}

	void
	on_joint_positions(const kalman_interfaces::msg::ArmValues::SharedPtr msg) {
		if (!kinematics_ready) {
			return;
		}
		if (msg->joints.size() < current_joint_positions.rows()) {
			RCLCPP_WARN(
			    get_logger(),
			    "Received joint position size (%zu) < expected (%d).",
			    msg->joints.size(),
			    current_joint_positions.rows()
			);
			return;
		}
		for (size_t i = 0; i < current_joint_positions.rows(); ++i) {
			current_joint_positions(i) = msg->joints[i];
		}
		joints_initialized = true;
	}

	geometry_msgs::msg::Twist transform_twist_to_base_frame(
	    const geometry_msgs::msg::TwistStamped::SharedPtr &msg
	) {

		geometry_msgs::msg::Twist base_twist = msg->twist;

		// Transform twist to base frame if frame_id is provided and not empty
		if (!msg->header.frame_id.empty() &&
		    msg->header.frame_id != base_link) {
			try {
				// Get transform from source frame to base frame
				geometry_msgs::msg::TransformStamped transform =
				    tf_buffer->lookupTransform(
				        base_link,
				        msg->header.frame_id,
				        msg->header.stamp,
				        rclcpp::Duration::from_nanoseconds(100000000)
				    ); // 100ms timeout

				tf2::Quaternion q(
				    transform.transform.rotation.x,
				    transform.transform.rotation.y,
				    transform.transform.rotation.z,
				    transform.transform.rotation.w
				);
				q.normalize();

				// Transform linear velocity
				tf2::Vector3 lin(
				    msg->twist.linear.x,
				    msg->twist.linear.y,
				    msg->twist.linear.z
				);
				tf2::Vector3 lin_base = tf2::quatRotate(q, lin);

				// Transform angular velocity (rvec)
				tf2::Vector3 ang(
				    msg->twist.angular.x,
				    msg->twist.angular.y,
				    msg->twist.angular.z
				);
				tf2::Vector3 ang_base = tf2::quatRotate(q, ang);

				base_twist.linear.x  = lin_base.x();
				base_twist.linear.y  = lin_base.y();
				base_twist.linear.z  = lin_base.z();
				base_twist.angular.x = ang_base.x();
				base_twist.angular.y = ang_base.y();
				base_twist.angular.z = ang_base.z();

				RCLCPP_DEBUG(
				    get_logger(),
				    "Transformed twist from frame '%s' to '%s'",
				    msg->header.frame_id.c_str(),
				    base_link.c_str()
				);
			} catch (const tf2::TransformException &ex) {
				RCLCPP_WARN_THROTTLE(
				    get_logger(),
				    *get_clock(),
				    1000,
				    "Could not transform twist from '%s' to '%s': %s. Using "
				    "twist as-is.",
				    msg->header.frame_id.c_str(),
				    base_link.c_str(),
				    ex.what()
				);
			}
		}

		return base_twist;
	}

	void
	on_target_twist(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
		std::lock_guard<std::mutex> lock(twist_mutex);
		latest_twist    = msg;
		last_twist_time = now();
	}

	// ----- Helpers for soft joint-limit avoidance & colinear-axis handling
	// -----

	KDL::JntArray computeLimitWeights(const KDL::JntArray &q) {
		KDL::JntArray w(q.rows());
		const double  eps  = 1e-3;
		// const double  gain = 4.0; // tune: higher => stronger avoidance
		const double  gain = 10.0; // tune: higher => stronger avoidance
		for (unsigned i = 0; i < q.rows(); ++i) {
			const double range = q_max_[i] - q_min_[i];
			if (range < 1e-6) {
				w(i) = 1.0;
				continue;
			}
			const double m_lo = q(i) - q_min_[i];
			const double m_hi = q_max_[i] - q(i);
			const double barrier =
			    (1.0 / std::max(m_lo, eps)) + (1.0 / std::max(m_hi, eps));
			double wi = 1.0 + gain * barrier * (1.0 / range);
			if (wi > 100.0) {
				wi = 100.0; // cap
			}
			w(i) = wi;
		}
		return w;
	}

	void boostWeightsForColinearAxes(const KDL::JntArray &q, KDL::JntArray &w) {
		if (!jac_solver) {
			return;
		}
		KDL::Jacobian J(arm_chain.getNrOfJoints());
		jac_solver->JntToJac(q, J);

		auto rotcol = [&](unsigned j) {
			return Eigen::Vector3d(J(3, j), J(4, j), J(5, j));
		};
		const double cos_thresh = 0.995; // ~ <5 deg from parallel/antiparallel

		for (unsigned a = 0; a < J.columns(); ++a) {
			Eigen::Vector3d za = rotcol(a);
			double          na = za.norm();
			if (na < 1e-9) {
				continue;
			}
			za /= na;
			for (unsigned b = a + 1; b < J.columns(); ++b) {
				Eigen::Vector3d zb = rotcol(b);
				double          nb = zb.norm();
				if (nb < 1e-9) {
					continue;
				}
				zb /= nb;

				double c = za.dot(zb);
				if (std::abs(c) > cos_thresh) {
					// Prefer the one farther from limits; penalize the one
					// closer to a bound
					auto proximity = [&](unsigned i) {
						const double mid  = q_mid_[i];
						const double half = 0.5 * (q_max_[i] - q_min_[i]);
						return std::min(
						    1.0, std::abs(q(i) - mid) / std::max(half, 1e-9)
						);
					};
					const double pa = proximity(a), pb = proximity(b);
					if (pa >= pb) {
						w(a) *= 2.0;
					} else {
						w(b) *= 2.0; // gentle bias
					}
				}
			}
		}

		// Optional: preserve your original biasing idea
		if (w.rows() >= 5) {
			w(3) *= 4.0;                       // damp joint 4
			w(4)  = std::max(0.3, w(4) * 0.7); // encourage joint 5 a bit
		}
	}

	// --------------------------------------------------------------------------

	void compute_joint_velocities() {
		if (!kinematics_ready || !joints_initialized) {
			return;
		}

		geometry_msgs::msg::TwistStamped::SharedPtr twist_msg;
		{
			std::lock_guard<std::mutex> lock(twist_mutex);
			if (!latest_twist ||
			    (now() - last_twist_time).seconds() > control_timeout) {
				return;
			}
			twist_msg = latest_twist;
		}

		// Build target twist in base frame
		geometry_msgs::msg::Twist base_twist =
		    transform_twist_to_base_frame(twist_msg);
		KDL::Twist target_twist;
		target_twist.vel.x(base_twist.linear.x);
		target_twist.vel.y(base_twist.linear.y);
		target_twist.vel.z(base_twist.linear.z);
		target_twist.rot.x(base_twist.angular.x);
		target_twist.rot.y(base_twist.angular.y);
		target_twist.rot.z(base_twist.angular.z);

		// Dynamic weights: joint-limit avoidance + colinear axes bias
		if (!q_min_.empty() &&
		    current_joint_positions.rows() == (int)q_min_.size()) {
			KDL::JntArray w = computeLimitWeights(current_joint_positions);
			boostWeightsForColinearAxes(current_joint_positions, w);
			ik_solver->setWeights(w);
			// Keep posture bias toward mid-range
			for (unsigned i = 0; i < q_nom.rows(); ++i) {
				q_nom(i) = q_mid_[i];
			}
			ik_solver->setOptPos(q_nom);
		}

		// IK velocity
		KDL::JntArray joint_velocities(current_joint_positions.rows());
		int           result = ik_solver->CartToJnt(
            current_joint_positions, target_twist, joint_velocities
        );
		if (result < 0) {
			RCLCPP_WARN_THROTTLE(
			    get_logger(),
			    *get_clock(),
			    1000,
			    "IK velocity solver failed (code %d)",
			    result
			);
			return;
		}

		// Global scaling (use absolute max)
		const double max_abs = joint_velocities.data.cwiseAbs().maxCoeff();
		if (max_abs > max_joint_vel && max_abs > 1e-9) {
			joint_velocities.data *= (max_joint_vel / max_abs);
		}

		// Limit-aware safety clip so next step can't cross bounds
		const double dt = (update_rate > 0.0) ? (1.0 / update_rate) : 0.0;
		if (dt > 0.0 && !q_min_.empty()) {
			const double margin = 0.9; // keep 10% away in one step
			for (unsigned i = 0; i < joint_velocities.rows(); ++i) {
				// Also hard-cap by global max (already scaled, but keep it
				// tight)
				if (joint_velocities(i) > max_joint_vel) {
					joint_velocities(i) = max_joint_vel;
				}
				if (joint_velocities(i) < -max_joint_vel) {
					joint_velocities(i) = -max_joint_vel;
				}

				const double v_hi =
				    (q_max_[i] - current_joint_positions(i)) / dt;
				const double v_lo =
				    (q_min_[i] - current_joint_positions(i)) / dt;
				joint_velocities(i) = std::min(
				    std::max(joint_velocities(i), margin * v_lo), margin * v_hi
				);
			}
		}

		// Publish
		auto vel_msg         = kalman_interfaces::msg::ArmValues();
		vel_msg.header.stamp = now();
		// vel_msg.joints.resize(joint_velocities.rows());
		for (size_t i = 0; i < joint_velocities.rows(); ++i) {
			vel_msg.joints[i] = joint_velocities(i);
		}
		joint_vel_pub->publish(vel_msg);
	}
};

} // namespace kalman_arm2

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_arm2::TwistIK)
