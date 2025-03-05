#include <deque>
#include <functional>
#include <rclcpp/logging.hpp>
#include <rclcpp/time.hpp>
#include <vector>

#include <opencv2/core/affine.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/quaternion.hpp>
#include <opencv2/core/types.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace kalman_slam {

static bool is_rclcpp_time_zero(const rclcpp::Time &time) {
	return time.seconds() == 0 && time.nanoseconds() == 0;
}

static cv::Affine3d
scale_affine(const cv::Affine3d &transform, double time_scale) {
	cv::Vec3d rvec = transform.rvec();
	cv::Vec3d tvec = transform.translation();
	return cv::Affine3d(rvec * time_scale, tvec * time_scale);
}

// A transformation between two stamped poses.
// Stored as velocity to allow time bounds to be altered while effectively
// trimming the time window - not stretching nor squishing it.
class Delta {
public:
	rclcpp::Time start_time;
	rclcpp::Time end_time;
	cv::Affine3d velocity;
};

// Subscribes nav_msgs/Odometry and assembles transform deltas.
// A delta is available only after the second valid message in sequence is
// received.
//
// Delta durations can be limited to somewhat-gracefully handle sensor dropouts.
// Normally the sensors should publish at a constant rate, and zeroed out
// messages should be published whenever the odometry is not available for
// whatever reason.
//
// The velocity in a delta is computed from the difference between the last two
// poses. Twist is not used as it can be vaguely interpreted and different
// packages can introduce single-frame delays or inexact euler angle
// mappings.
class DeltaSubscriber {
	rclcpp::Node *node;

	std::function<void(Delta)> callback;
	double                     max_duration;

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;

	rclcpp::Time last_time;
	cv::Affine3d last_pose;

public:
	DeltaSubscriber(
	    rclcpp::Node                     *node,
	    const std::string                &topic,
	    const std::function<void(Delta)> &callback,
	    double                            max_duration = 1.0
	)
	    : node(node), callback(callback), max_duration(max_duration) {
		sub = node->create_subscription<nav_msgs::msg::Odometry>(
		    topic,
		    10,
		    std::bind(&DeltaSubscriber::odom_cb, this, std::placeholders::_1)
		);
	}

	void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
		// Skip nulls
		if (is_pose_zero(msg->pose.pose)) {
			last_time = rclcpp::Time();
			// ^ reset last_time so that the next valid message will be saved as
			// the initial and transforms will be computed starting from it
			return;
		}

		// If this is the first message, just store it.
		if (is_rclcpp_time_zero(last_time)) {
			last_time = msg->header.stamp;
			last_pose = affine_from_pose(msg->pose.pose);
			return;
		}

		// Verify that the message is from the future.
		if (rclcpp::Time(msg->header.stamp) < last_time) {
			RCLCPP_WARN(
			    node->get_logger(), "Received message from the past. Ignoring."
			);
			return;
		}

		// Compute start/end time of the delta transform.
		rclcpp::Time start_time = last_time;
		rclcpp::Time end_time   = msg->header.stamp;
		if (end_time - start_time >
		    rclcpp::Duration::from_seconds(max_duration)) {
			// apply max_duration
			start_time =
			    end_time - rclcpp::Duration::from_seconds(max_duration);
		}

		// Compute velocity.
		cv::Affine3d new_pose        = affine_from_pose(msg->pose.pose);
		cv::Affine3d delta_transform = last_pose.inv() * new_pose;
		cv::Affine3d velocity        = scale_affine(
            delta_transform, 1 / (end_time - start_time).seconds()
        );

		// Call back.
		callback({start_time, end_time, velocity});

		// Update last time and pose.
		last_time = end_time;
		last_pose = new_pose;
	}

	cv::Affine3d affine_from_pose(const geometry_msgs::msg::Pose &pose) {
		cv::Vec3d translation(
		    pose.position.x, pose.position.y, pose.position.z
		);
		cv::Quatd rotation(
		    pose.orientation.w,
		    pose.orientation.x,
		    pose.orientation.y,
		    pose.orientation.z
		);
		return cv::Affine3d(rotation.toRotVec(), translation);
	}

	bool is_pose_zero(const geometry_msgs::msg::Pose &pose) {
		return pose.position.x == 0 && pose.position.y == 0 &&
		       pose.position.z == 0 && pose.orientation.x == 0 &&
		       pose.orientation.y == 0 && pose.orientation.z == 0;
		// ^ orientation.w is not considered because it might be 1 for initial
		// readings and then 0 for nulls.
	}
};

// Integrates odometry deltas.
//
// Keeps track of a buffer of odometry deltas:
// >(   )   (      )   ( )(   ) ()
// > ( ) (  ) ()   (     )  ( )( )
// ^ where each () is a delta with start/end time bounds.
// ^ where each line is a different odom source.
//
// The buffer always begins where the last integration step ended.
//
// Once a new delta is received, it is pushed to the buffer and the integration
// loop is run. The loop goes through all deltas in the buffer and integrates
// them until it reaches a point where not all readings are yet available.
// A reading can be unavailable in a situation with multiple sensors, where all
// readings have been integrated recently and a new one has just arrived. In
// this case we want to wait for readings from other sensors to arrive in order
// to use their average velocity for more precise integration.
class DeadReckoning : public rclcpp::Node {
public:
	std::vector<DeltaSubscriber>                          odom_subs;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
	tf2_ros::TransformBroadcaster                         tf_broadcaster;

	double timeout;

	cv::Affine3d integrated_transform;

	std::vector<std::deque<Delta>> deltas;
	std::vector<rclcpp::Time>      last_reading_times;

	DeadReckoning(const rclcpp::NodeOptions &options)
	    : Node("dead_reckoning", options), tf_broadcaster(this) {
		declare_parameter("odom_frame_id", "odom");
		declare_parameter("base_link_frame_id", "base_link");
		declare_parameter("publish_tf", true);
		declare_parameter("output_covariance", 0.1);

		// Subscribe to visodo
		int num_odoms = declare_parameter("num_odoms", 1);
		timeout       = declare_parameter("timeout", 0.3);
		odom_subs.reserve(num_odoms);
		// ^ Required! DeltaSubscriber binds "this" to a callback, and it must
		// not be invalidated by a reallocation.
		for (int i = 0; i < num_odoms; i++) {
			odom_subs.emplace_back(
			    this,
			    "odom" + std::to_string(i),
			    [this, i](Delta delta) {
				    delta_cb(delta, i);
			    },
			    timeout
			);
		}

		// Publish integrated odometry
		odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);

		// Initialize state
		deltas.resize(num_odoms);
		last_reading_times.resize(num_odoms);
		integrated_transform = cv::Affine3d::Identity();
	}

	void delta_cb(Delta delta, size_t i) {
		RCLCPP_DEBUG(get_logger(), "Received delta from odom%d", (int)i);

		// Find start time of the buffer.
		rclcpp::Time start_time;
		for (const auto &d : deltas) {
			if (!d.empty()) {
				if (is_rclcpp_time_zero(start_time)) {
					start_time = d.front().start_time;
				} else {
					start_time = std::min(start_time, d.front().start_time);
				}
			}
		}

		// Trim the delta before adding to the buffer.
		// Set the start_time of the delta to begin at the start of the buffer.
		// If in the process the delta gains negative duration, it is discarded.
		if (!is_rclcpp_time_zero(start_time) && delta.start_time < start_time) {
			delta.start_time = start_time;
			if (delta.start_time >= delta.end_time) {
				// This delta is too old and was made invalid.
				return;
			}
		}

		// Push to buffer and save last reading time.
		deltas[i].push_back(delta);
		last_reading_times[i] = delta.end_time;

		// Init start_time if it was not set before pushing.
		if (is_rclcpp_time_zero(start_time)) {
			start_time = delta.start_time;
		}

		// Select the end time of the buffer (= current time).
		rclcpp::Time end_time;
		for (const auto &d : deltas) {
			if (!d.empty()) {
				if (is_rclcpp_time_zero(end_time)) {
					end_time = d.back().end_time;
				} else {
					end_time = std::max(end_time, d.back().end_time);
				}
			}
		}
		// ^ end_time is guaranteed to be set here as we've just pushed.

		// Integration loop.
		// We go through as many deltas as we can until we have to wait for
		// more readings.
		bool         integrated_anything = false;
		rclcpp::Time end_time_of_integration;
		while (true) {
			// Find the next integration time.
			// This will be the first start/end time in the buffer.
			// Only times > buffer's start_time are considered so that we get
			// duration > 0.
			rclcpp::Time next_start_time = end_time;
			for (const auto &d : deltas) {
				if (!d.empty()) {
					if (d.front().start_time > start_time) {
						next_start_time =
						    std::min(next_start_time, d.front().start_time);
					}
					if (d.front().end_time > start_time) {
						next_start_time =
						    std::min(next_start_time, d.front().end_time);
					}
				}
			}

			// See if we have to stop here to wait for more readings.
			bool can_integrate = true;
			for (int j = 0; j < deltas.size(); j++) {
				auto &d = deltas[j];
				// See if we would have to wait for this source.
				// We have to wait if there are no deltas and the sensor has not
				// timed out.
				if (d.empty() && !is_rclcpp_time_zero(last_reading_times[j]) &&
				    end_time - last_reading_times[j] <
				        rclcpp::Duration::from_seconds(timeout)) {
					can_integrate = false;
					break;
				}
			}
			if (!can_integrate) {
				break;
			}

			// Integrate.
			// Average the velocities of deltas active during the integration
			// window.
			std::vector<cv::Affine3d> delta_transforms;
			for (const auto &d : deltas) {
				if (d.empty() || d.front().end_time <= start_time ||
				    d.front().start_time >= next_start_time) {
					continue;
				}
				cv::Affine3d delta_transform = scale_affine(
				    d.front().velocity, (next_start_time - start_time).seconds()
				);
				delta_transforms.push_back(delta_transform);
			}
			if (delta_transforms.size() > 0) {
				cv::Affine3d mean_transform = mean_affine(delta_transforms);
				integrated_transform = integrated_transform * mean_transform;
			}
			integrated_anything     = true;
			end_time_of_integration = next_start_time;

			RCLCPP_DEBUG(
			    get_logger(),
			    "Integrated deltas from %d sources over %f seconds:",
			    (int)delta_transforms.size(),
			    (next_start_time - start_time).seconds()
			);

			// log delta_transforms
			for (size_t j = 0; j < delta_transforms.size(); j++) {
				cv::Affine3d d = delta_transforms[j];
				// extract euler angles from delta rot
				cv::Vec3d t   = d.translation();
				cv::Quat  rot = cv::Quatd::createFromRvec(d.rvec());
				cv::Vec3d euler =
				    rot.toEulerAngles(cv::QuatEnum::EulerAnglesType::INT_XYZ);
				RCLCPP_DEBUG(
				    get_logger(),
				    "  odom%d: X=%f, Y=%f, Z=%f ; R=%f, P=%f, Y=%f",
				    (int)j,
				    t[0],
				    t[1],
				    t[2],
				    euler[0],
				    euler[1],
				    euler[2]
				);
			}

			// Clip readings to begin at the new buffer start_time.
			for (auto &d : deltas) {
				while (!d.empty()) {
					if (d.front().start_time <= next_start_time) {
						d.front().start_time = next_start_time;
						if (d.front().start_time >= d.front().end_time) {
							// This delta is too old and was made invalid.
							d.pop_front();
						} else {
							break;
						}
					} else {
						break;
					}
				}
			}

			// Update start time.
			start_time = next_start_time;
		}

		// Publish transforms.
		if (integrated_anything) {
			// Publish odom.
			nav_msgs::msg::Odometry odom_msg =
			    odom_from_affine(integrated_transform, end_time_of_integration);
			odom_pub->publish(odom_msg);

			// Publish TF.
			if (get_parameter("publish_tf").as_bool()) {
				geometry_msgs::msg::TransformStamped tf_msg =
				    transform_from_affine(
				        integrated_transform, end_time_of_integration
				    );
				tf_broadcaster.sendTransform(tf_msg);
			}
		}
	}

	geometry_msgs::msg::TransformStamped transform_from_affine(
	    const cv::Affine3d &affine, const rclcpp::Time &time
	) {
		// Extract translation and rotation
		cv::Vec3d translation = affine.translation();
		cv::Quatd rotation    = cv::Quatd::createFromRotMat(affine.rotation());

		// Create and publish transform
		geometry_msgs::msg::TransformStamped msg;
		msg.header.stamp    = time;
		msg.header.frame_id = get_parameter("odom_frame_id").as_string();
		msg.child_frame_id  = get_parameter("base_link_frame_id").as_string();

		// Set translation
		msg.transform.translation.x = translation[0];
		msg.transform.translation.y = translation[1];
		msg.transform.translation.z = translation[2];

		// Set rotation
		msg.transform.rotation.w = rotation.w;
		msg.transform.rotation.x = rotation.x;
		msg.transform.rotation.y = rotation.y;
		msg.transform.rotation.z = rotation.z;

		return msg;
	}

	nav_msgs::msg::Odometry
	odom_from_affine(const cv::Affine3d &affine, const rclcpp::Time &time) {
		// Extract translation and rotation
		cv::Vec3d translation = affine.translation();
		cv::Quatd rotation    = cv::Quatd::createFromRotMat(affine.rotation());

		// Create and publish transform
		nav_msgs::msg::Odometry msg;
		msg.header.stamp    = time;
		msg.header.frame_id = get_parameter("odom_frame_id").as_string();
		msg.child_frame_id  = get_parameter("base_link_frame_id").as_string();

		// Set translation
		msg.pose.pose.position.x = translation[0];
		msg.pose.pose.position.y = translation[1];
		msg.pose.pose.position.z = translation[2];

		// Set rotation
		msg.pose.pose.orientation.w = rotation.w;
		msg.pose.pose.orientation.x = rotation.x;
		msg.pose.pose.orientation.y = rotation.y;
		msg.pose.pose.orientation.z = rotation.z;

		// Set covariance
		double cov             = get_parameter("output_covariance").as_double();
		msg.pose.covariance[0] = cov;
		msg.pose.covariance[7] = cov;
		msg.pose.covariance[14] = cov;
		msg.pose.covariance[21] = cov;
		msg.pose.covariance[28] = cov;
		msg.pose.covariance[35] = cov;

		return msg;
	}

	cv::Affine3d mean_affine(const std::vector<cv::Affine3d> &transforms) {
		// Compute mean translation
		cv::Vec3d mean_translation(0, 0, 0);
		for (const auto &t : transforms) {
			mean_translation += t.translation();
		}
		mean_translation /= static_cast<double>(transforms.size());

		// Compute mean rotation
		cv::Vec3d mean_rotation(0, 0, 0);
		for (const auto &t : transforms) {
			mean_rotation += t.rvec();
		}
		mean_rotation /= static_cast<double>(transforms.size());

		return cv::Affine3d(mean_rotation, mean_translation);
	}
};

} // namespace kalman_slam

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_slam::DeadReckoning)
