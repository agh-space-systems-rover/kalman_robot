#include <aruco_opencv_msgs/msg/aruco_detection.hpp>
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/basic_types.h>
#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include <cstdint>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <sys/types.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <thread>

struct UVAnchor {
	std::string     tf_name; // e.g. "aruco_12"
	Eigen::Vector2d uv;      // your planar coords (any units)
};

class BuildUV : public BT::StatefulActionNode {
  public:
	BuildUV(
	    const std::string           &name,
	    const BT::NodeConfiguration &config,
	    rclcpp::Node                *parent
	);

	static BT::PortsList providedPorts();

	BT::NodeStatus onStart() override;
	BT::NodeStatus onRunning() override;
	void           onHalted() override;

  private:
	rclcpp::Node *parent_;

	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr arm_pub_;
	std::unique_ptr<tf2_ros::Buffer>                               tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener>                    tf_listener_;
	std::shared_ptr<tf2_ros::TransformBroadcaster>       tf_broadcaster_;
	std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

	bool buildUnitScale(
	    const std::vector<UVAnchor> &anchors,
	    tf2_ros::Buffer             &tf_buffer,
	    const std::string           &base_frame,
	    const std::string           &uv_board_frame,
	    Eigen::Isometry3d           &T_base_to_board, // Output
	    rclcpp::Logger logger = rclcpp::get_logger("UVtoXYZMapper")
	) {
		// Eigen::Isometry3d T_base_to_board{Eigen::Isometry3d::Identity()};
		if (anchors.size() < 3) {
			RCLCPP_ERROR(logger, "Need >=3 anchors, got %zu", anchors.size());
			return false;
		}

		// 1) Fetch 3D anchor positions in base_frame
		const int                    N = static_cast<int>(anchors.size());
		std::vector<Eigen::Vector3d> P;
		P.reserve(N);
		Eigen::Matrix<double, 2, Eigen::Dynamic> UV(2, N);
		try {
			for (int i = 0; i < N; ++i) {
				const auto &a  = anchors[i];
				auto        ts = tf_buffer.lookupTransform(
                    base_frame, a.tf_name, tf2::TimePointZero
                );
				Eigen::Isometry3d T = tf2::transformToEigen(ts);
				P.emplace_back(T.translation());
				UV(0, i) = a.uv.x();
				UV(1, i) = a.uv.y();
			}
		} catch (const tf2::TransformException &e) {
			RCLCPP_ERROR(logger, "TF lookup failed: %s", e.what());
			return false;
		}

		// 2) Plane fit by PCA
		Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
		for (auto &p : P) {
			centroid += p;
		}
		centroid /= double(N);

		Eigen::Matrix3d C = Eigen::Matrix3d::Zero();
		for (auto &p : P) {
			Eigen::Vector3d d  = p - centroid;
			C                 += d * d.transpose();
		}

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
		if (es.info() != Eigen::Success) {
			RCLCPP_ERROR(logger, "Eigen decomposition failed");
			return false;
		}
		// Smallest eigenvector = normal
		Eigen::Vector3d n_hat = es.eigenvectors().col(0).normalized();
		// Largest eigenvector = dominant in-plane axis
		Eigen::Vector3d e1 = es.eigenvectors().col(2).normalized();
		// Build orthonormal in-plane basis
		Eigen::Vector3d e2 = n_hat.cross(e1).normalized();

		// 3) Project 3D anchors to plane coords S (2xN) in meters
		Eigen::Matrix<double, 2, Eigen::Dynamic> S(2, N);
		for (int i = 0; i < N; ++i) {
			Eigen::Vector3d d = P[i] - centroid;
			S(0, i)           = e1.dot(d);
			S(1, i)           = e2.dot(d);
		}

		// 4) 2D Procrustes (rotation+translation only): UV -> S
		Eigen::Vector2d Umean = UV.rowwise().mean();
		Eigen::Vector2d Smean = S.rowwise().mean();
		Eigen::MatrixXd U0    = UV.colwise() - Umean;
		Eigen::MatrixXd S0    = S.colwise() - Smean;

		Eigen::Matrix2d                   H = S0 * U0.transpose();
		Eigen::JacobiSVD<Eigen::Matrix2d> svd(
		    H, Eigen::ComputeFullU | Eigen::ComputeFullV
		);
		Eigen::Matrix2d U  = svd.matrixU();
		Eigen::Matrix2d V  = svd.matrixV();
		Eigen::Matrix2d R2 = U * V.transpose();
		if (R2.determinant() < 0) { // reflection fix
			U.col(1) *= -1;
			R2        = U * V.transpose();
		}
		Eigen::Vector2d t2 = Smean - R2 * Umean;

		// 5) Build 3D mapping columns (unit length, orthonormal)
		Eigen::Vector3d U_vec = e1 * R2(0, 0) + e2 * R2(1, 0); // +1 u in meters
		Eigen::Vector3d V_vec = e1 * R2(0, 1) + e2 * R2(1, 1); // +1 v in meters
		Eigen::Vector3d O3 =
		    centroid + e1 * t2.x() + e2 * t2.y(); // uv origin in base

		// Ensure right-handed and orthonormal (guard against tiny numerical
		// drift)
		U_vec.normalize();
		V_vec = (n_hat.cross(U_vec))
		            .normalized(); // make v orthogonal to u using normal
		n_hat = U_vec.cross(V_vec).normalized();

		// 6) Pack into transform (base -> uv_board)
		// base_frame_    = base_frame;
		// uv_board_frame_= uv_board_frame;

		Eigen::Matrix3d R;
		R.col(0) = U_vec; // x-axis = +u
		R.col(1) = V_vec; // y-axis = +v
		R.col(2) = n_hat; // z-axis = plane normal

		T_base_to_board.linear()      = R;
		T_base_to_board.translation() = O3;

		// 7) Store A (for optional direct mapping) and RMS
		// A_.col(0) = U_vec; A_.col(1) = V_vec; A_.col(2) = O3;

		double err2 = 0.0;
		for (int i = 0; i < N; ++i) {
			Eigen::Vector3d pred  = O3 + U_vec * UV(0, i) + V_vec * UV(1, i);
			err2                 += (pred - P[i]).squaredNorm();
		}
		double rms_   = std::sqrt(err2 / N);
		bool   valid_ = true;
		return true;
	}
};
