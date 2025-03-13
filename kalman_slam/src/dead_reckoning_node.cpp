#include <deque>
#include <functional>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/subscription.hpp>
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
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

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
	class ImuReading {
	  public:
		rclcpp::Time time;
		cv::Quatd    rot;

		ImuReading(rclcpp::Time time, cv::Quatd rot) : time(time), rot(rot) {}
	};

	std::vector<DeltaSubscriber>                           odom_subs;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr  odom_pub;
	tf2_ros::TransformBroadcaster                          tf_broadcaster;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
	tf2_ros::Buffer                                        tf_buffer;
	tf2_ros::TransformListener                             tf_listener;
	rclcpp::TimerBase::SharedPtr                           imu_corr_timer;

	double timeout;

	cv::Affine3d integrated_transform;
	rclcpp::Time last_integration_time;

	std::vector<std::deque<Delta>> deltas;
	std::vector<rclcpp::Time>      last_reading_times;

	std::deque<ImuReading> imu_b2w_buffer, transform_b2o_buffer;

	DeadReckoning(const rclcpp::NodeOptions &options)
	    : Node("dead_reckoning", options), tf_broadcaster(this),
	      tf_buffer(get_clock()), tf_listener(tf_buffer) {
		declare_parameter("odom_frame_id", "odom");
		declare_parameter("base_link_frame_id", "base_link");
		declare_parameter("publish_tf", true);
		declare_parameter("sync_with_odom", -1);
		declare_parameter("output_covariance", 0.1);
		declare_parameter("imu_correction_window_duration", 1.0);
		declare_parameter("imu_correction_window_samples", 30);
		declare_parameter("imu_correction_kp", 0.2);
		declare_parameter("imu_correction_rate", 10.0);
		declare_parameter("imu_correction_use_yaw", false);

		// Subscribe to visodo
		int num_odoms = declare_parameter("num_odoms", 1);
		timeout       = declare_parameter("timeout", 0.5);
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

		// Subscribe to IMU
		imu_sub = create_subscription<sensor_msgs::msg::Imu>(
		    "imu",
		    10,
		    std::bind(&DeadReckoning::imu_cb, this, std::placeholders::_1)
		);

		// Timer for IMU corrections
		imu_corr_timer = create_wall_timer(
		    std::chrono::duration<double>(
		        1.0 / get_parameter("imu_correction_rate").as_double()
		    ),
		    std::bind(&DeadReckoning::imu_corr_timer_cb, this)
		);

		// Initialize state
		deltas.resize(num_odoms);
		last_reading_times.resize(num_odoms);
		integrated_transform = cv::Affine3d::Identity();
	}

	void delta_cb(Delta delta, size_t i) {
		RCLCPP_DEBUG(get_logger(), "Received delta from odom%d", (int)i);
		int sync_with_odom = get_parameter("sync_with_odom").as_int();

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
		// Also trim to start after last_integration_time.
		if (!is_rclcpp_time_zero(last_integration_time) &&
		    delta.start_time < last_integration_time) {
			delta.start_time = last_integration_time;
			if (delta.start_time >= delta.end_time) {
				// This delta is too old and was made invalid.
				return;
			}
		}

		// Push to buffer and save last reading time.
		deltas[i].push_back(delta);
		last_reading_times[i] = delta.end_time;

		// // DEBUG: Log the contents of the buffer. Just timestamps.
		// RCLCPP_DEBUG(
		//     get_logger(), "Buffer contents after pushing odom%d:", (int)i
		// );
		// for (int j = 0; j < deltas.size(); j++) {
		// 	auto &ds = deltas[j];
		// 	if (ds.empty()) {
		// 		RCLCPP_DEBUG(get_logger(), "  %d: empty", j);
		// 	} else {
		// 		RCLCPP_DEBUG(
		// 		    get_logger(),
		// 		    "  %d: %f -> %f",
		// 		    j,
		// 		    ds.front().start_time.seconds(),
		// 		    ds.back().end_time.seconds()
		// 		);
		// 	}
		// }

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
		bool integrated_anything = false;
		while (true) {
			// Find the next integration time.
			// This will be the first start/end time in the buffer.
			// Only times > buffer's start_time are considered so that we
			// get duration > 0.
			rclcpp::Time next_start_time = end_time;
			for (int j = 0; j < deltas.size(); j++) {
				auto &d = deltas[j];
				if (!d.empty()) {
					if (d.front().start_time > start_time) {
						if (d.front().start_time <= next_start_time) {
							next_start_time = d.front().start_time;
						}
					}
					if (d.front().end_time > start_time) {
						if (d.front().end_time <= next_start_time) {
							next_start_time = d.front().end_time;
						}
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
			std::vector<int>          delta_transforms_i;
			for (int j = 0; j < deltas.size(); j++) {
				auto &d = deltas[j];
				if (d.empty() || d.front().end_time <= start_time ||
				    d.front().start_time >= next_start_time) {
					continue;
				}
				cv::Affine3d delta_transform = scale_affine(
				    d.front().velocity, (next_start_time - start_time).seconds()
				);
				delta_transforms.push_back(delta_transform);
				delta_transforms_i.push_back(j);
			}
			if (delta_transforms.size() > 0) {
				cv::Affine3d mean_transform = mean_affine(delta_transforms);
				integrated_transform = integrated_transform * mean_transform;
			}
			integrated_anything   = true;
			last_integration_time = next_start_time;

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
				    delta_transforms_i[j],
				    t[0],
				    t[1],
				    t[2],
				    euler[0],
				    euler[1],
				    euler[2]
				);
			}

			// Clip readings to begin at the new buffer start_time.
			for (int j = 0; j < deltas.size(); j++) {
				auto &d = deltas[j];
				while (!d.empty()) {
					if (d.front().start_time <= next_start_time) {
						d.front().start_time = next_start_time;
						if (d.front().start_time >= d.front().end_time) {
							// This delta is too old and was made invalid.
							d.pop_front();

							// Publish if sync_with_odom is set and the
							// corresponding odom has just been fully
							// integrated.
							if (sync_with_odom == j) {
								publish_transform(last_integration_time);
							}
						} else {
							break;
						}
					} else {
						break;
					}
				}
			}

			// Push to transform buffer for IMU corrections.
			transform_b2o_buffer.emplace_back(
			    last_integration_time,
			    cv::Quatd::createFromRvec(integrated_transform.rvec())
			);
			while (!transform_b2o_buffer.empty() &&
			       last_integration_time - transform_b2o_buffer.front().time >
			           rclcpp::Duration::from_seconds(
			               get_parameter("imu_correction_window_duration")
			                   .as_double()
			           )) {
				transform_b2o_buffer.pop_front();
			}

			// Update start time.
			start_time = next_start_time;
		}

		// Publish transform. (Only if sync_with_odom is not set.)
		if (integrated_anything && sync_with_odom == -1) {
			publish_transform(last_integration_time);
		}
	}

	void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
		// Transform to base frame rotation.
		// 1. Get transform from base frame to IMU frame.
		cv::Affine3d base_to_imu_transform;
		try {
			geometry_msgs::msg::TransformStamped tf = tf_buffer.lookupTransform(
			    msg->header.frame_id,
			    get_parameter("base_link_frame_id").as_string(),
			    rclcpp::Time(msg->header.stamp)
			);
			base_to_imu_transform = cv::Affine3d(
			    cv::Quatd(
			        tf.transform.rotation.w,
			        tf.transform.rotation.x,
			        tf.transform.rotation.y,
			        tf.transform.rotation.z
			    )
			        .toRotVec(),
			    cv::Vec3d(
			        tf.transform.translation.x,
			        tf.transform.translation.y,
			        tf.transform.translation.z
			    )
			);
		} catch (tf2::TransformException &ex) {
			RCLCPP_ERROR(
			    get_logger(), "Could not transform IMU reading: %s", ex.what()
			);
			return;
		}
		// 2. Transform IMU orientation to base frame.
		cv::Quatd imu_to_world(
		    msg->orientation.w,
		    msg->orientation.x,
		    msg->orientation.y,
		    msg->orientation.z
		);
		cv::Quatd base_to_imu =
		    cv::Quatd::createFromRvec(base_to_imu_transform.rvec());
		cv::Quatd base_to_world = imu_to_world * base_to_imu;
		// ^ rotation of base frame in the world frame

		// Push to buffer.
		imu_b2w_buffer.emplace_back(msg->header.stamp, base_to_world);
		while (
		    !imu_b2w_buffer.empty() &&
		    rclcpp::Time(msg->header.stamp) - imu_b2w_buffer.front().time >
		        rclcpp::Duration::from_seconds(
		            get_parameter("imu_correction_window_duration").as_double()
		        )
		) {
			imu_b2w_buffer.pop_front();
		}
	}

	void imu_corr_timer_cb() {
		// Check if buffers have enough data.
		if (imu_b2w_buffer.size() < 2 || transform_b2o_buffer.size() < 2) {
			return;
		}

		// Pick common start/end time of the window
		rclcpp::Time start_time = std::max(
		    imu_b2w_buffer.front().time, transform_b2o_buffer.front().time
		);
		rclcpp::Time end_time = std::min(
		    imu_b2w_buffer.back().time, transform_b2o_buffer.back().time
		);
		if (end_time - start_time <
		    rclcpp::Duration::from_seconds(
		        get_parameter("imu_correction_window_duration").as_double() / 2
		    )) {
			return;
		}

		// Compute correction.
		bool      use_yaw = get_parameter("imu_correction_use_yaw").as_bool();
		cv::Quatd mean_o2w;
		if (use_yaw) {
			mean_o2w = calc_imu_correction(start_time, end_time);
		} else {
			mean_o2w = calc_imu_correction_no_yaw(start_time, end_time);
		}

		// Apply correction to integrated_transform.
		// integrated_transform = base -> odom
		// we want it to be base -> world
		cv::Quatd new_rot_b2w =
		    mean_o2w * cv::Quatd::createFromRvec(integrated_transform.rvec());
		integrated_transform = cv::Affine3d(
		    new_rot_b2w.toRotVec(), integrated_transform.translation()
		);
		// Apply correction to the transform buffer.
		for (auto &r : transform_b2o_buffer) {
			r.rot = mean_o2w * r.rot;
		}

		// Log the correction.
		cv::Vec3d euler =
		    mean_o2w.toEulerAngles(cv::QuatEnum::EulerAnglesType::INT_XYZ);
		RCLCPP_DEBUG(
		    get_logger(),
		    "Applied IMU correction: dR=%f, dP=%f, dY=%f",
		    euler[0],
		    euler[1],
		    euler[2]
		);
	}

	cv::Quatd calc_imu_correction(
	    const rclcpp::Time &start_time, const rclcpp::Time &end_time
	) {
		// Go over both sequences and compute mean correction (odom <-> world
		// rotation).
		cv::Vec3d mean_o2w_rvec(0, 0, 0);
		size_t    num_seq_samples =
		    get_parameter("imu_correction_window_samples").as_int();
		for (size_t i = 0; i < num_seq_samples; i++) {
			// Select the sampling time.
			rclcpp::Time sample_time =
			    start_time + rclcpp::Duration::from_seconds(
			                     i * (end_time - start_time).seconds() /
			                     (num_seq_samples - 1)
			                 );

			// Take samples from both sequences.
			cv::Quatd b2w = resample_quat_sequence(imu_b2w_buffer, sample_time);
			cv::Quatd b2o =
			    resample_quat_sequence(transform_b2o_buffer, sample_time);

			// Compute odom frame to world frame rotation.
			cv::Quatd o2w = b2w * b2o.inv();

			// Accumulate.
			mean_o2w_rvec += norm_rvec(o2w.toRotVec());
		}
		mean_o2w_rvec      /= static_cast<double>(num_seq_samples);
		mean_o2w_rvec      *= get_parameter("imu_correction_kp").as_double();
		cv::Quatd mean_o2w  = cv::Quatd::createFromRvec(mean_o2w_rvec);

		return mean_o2w;
	}

	cv::Quatd calc_imu_correction_no_yaw(
	    const rclcpp::Time &start_time, const rclcpp::Time &end_time
	) {
		// Go over both sequences and compute mean gravity direction.
		const cv::Vec3d g_w(0, 0, -1);
		cv::Vec3d       mean_g_o(0, 0, 0);
		// ^ mean gravity vector in odom frame
		size_t num_seq_samples =
		    get_parameter("imu_correction_window_samples").as_int();
		for (size_t i = 0; i < num_seq_samples; i++) {
			// Select the sampling time.
			rclcpp::Time sample_time =
			    start_time + rclcpp::Duration::from_seconds(
			                     i * (end_time - start_time).seconds() /
			                     (num_seq_samples - 1)
			                 );

			// Take samples from both sequences.
			cv::Quatd b2w = resample_quat_sequence(imu_b2w_buffer, sample_time);
			cv::Quatd b2o =
			    resample_quat_sequence(transform_b2o_buffer, sample_time);

			// Compute gravity vector delta.
			cv::Quatd w2o = b2o * b2w.inv();
			cv::Vec3d g_o = w2o.toRotMat3x3() * g_w;

			// Accumulate.
			mean_g_o += g_o;
		}
		mean_g_o /= static_cast<double>(num_seq_samples);

		// Compute correction quaternion.
		cv::Vec3d axis_o2w  = mean_g_o.cross(g_w);
		double    angle_ow  = std::acos(mean_g_o.dot(g_w));
		angle_ow           *= get_parameter("imu_correction_kp").as_double();
		cv::Quatd mean_o2w = cv::Quatd::createFromAngleAxis(angle_ow, axis_o2w);

		return mean_o2w;
	}

	void publish_transform(rclcpp::Time time) {
		// Publish odom.
		nav_msgs::msg::Odometry odom_msg =
		    odom_from_affine(integrated_transform, time);
		odom_pub->publish(odom_msg);

		// Publish TF.
		if (get_parameter("publish_tf").as_bool()) {
			geometry_msgs::msg::TransformStamped tf_msg =
			    transform_from_affine(integrated_transform, time);
			tf_broadcaster.sendTransform(tf_msg);
		}

		RCLCPP_DEBUG(
		    get_logger(), "Published odom and TF at %f", time.seconds()
		);
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

	cv::Vec3d norm_rvec(const cv::Vec3d &rvec) {
		double mag = cv::norm(rvec);
		if (mag > CV_PI) {
			return rvec * (1 - 2 * CV_PI / mag);
		} else {
			return rvec;
		}
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
			mean_rotation += norm_rvec(t.rvec());
		}
		mean_rotation /= static_cast<double>(transforms.size());

		return cv::Affine3d(mean_rotation, mean_translation);
	}

	cv::Quatd resample_quat_sequence(
	    const std::deque<ImuReading> &sequence, const rclcpp::Time &time
	) {
		// Find the two closest readings.
		size_t i_prev = 0;
		for (size_t i = 0; i < sequence.size(); i++) {
			if (sequence[i].time > time) {
				break;
			}
			i_prev = i;
		}
		if (i_prev == sequence.size() - 1) {
			return sequence.back().rot;
		}
		size_t i_next = i_prev + 1;

		// Compute mix factor from time difference.
		float mix_factor =
		    (time - sequence[i_prev].time).seconds() /
		    (sequence[i_next].time - sequence[i_prev].time).seconds();

		// Interpolate.
		return cv::Quatd::slerp(
		    sequence[i_prev].rot, sequence[i_next].rot, mix_factor
		);
	}
};

} // namespace kalman_slam

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_slam::DeadReckoning)
