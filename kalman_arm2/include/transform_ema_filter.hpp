#pragma once

#include <algorithm>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Transform.hpp>

/**
 * @brief Exponential moving average filter for tf2::Transform
 *
 * @display_description Filters a stream of 3D transforms, useful for smoothing
 * noisy pose estimates from sensors or vision systems.
 *
 * This filter blends successive tf2::Transform samples using an exponential
 * moving average. The filter keeps track of the current state, the smoothing
 * factor `alpha`, and the number of samples processed.
 *
 * @note The class is intended to be used in a thread‑safe environment. All
 * member functions are lightweight and re‑entrant except `filter`, which
 * updates internal state.
 *
 * @mode Public API with the following functions:
 *   - setAlpha(double)
 *   - alpha()
 *   - reset()
 *   - state()
 *   - sampleCount()
 *   - filter(const tf2::Transform&)
 */
class TransformEmaFilter {
  public:
	explicit TransformEmaFilter(double alpha = 0.2)
	    : alpha_(clampAlpha(alpha)), initialized_(false) {}

	/**
	 * @brief Set the smoothing factor.
	 *
	 * @param a New alpha value in (0, 1]. Values outside the
	 *          range are clamped to the nearest valid value.
	 */
	void setAlpha(double a) {
		alpha_ = clampAlpha(a);
	}

	/** @brief Current alpha value */
	double alpha() const {
		return alpha_;
	}

	/** @brief Reset filter state. */
	void reset() {
		initialized_ = false;
	}

	/** @brief Return current filtered transform. */
	const tf2::Transform &state() const {
		return state_;
	}

	/** @brief Number of samples processed. */
	size_t sampleCount() const {
		return samples_;
	}

	/**
	 * @brief Process a new sample and return the filtered result.
	 *
	 * The first call simply stores the sample. Subsequent calls
	 * blend translation using an EMA and rotation using a SLERP
	 * with a hemisphere‑fix to ensure the shortest quaternion
	 * path.
	 *
	 * @param sample New transform sample.
	 * @return The updated filtered transform.
	 */
	tf2::Transform filter(const tf2::Transform &sample) {
		if (!initialized_) {
			state_       = sample;
			initialized_ = true;
			return state_;
		}
		++samples_;

		// Blend translation
		const tf2::Vector3 x_prev = state_.getOrigin();
		const tf2::Vector3 x_new  = sample.getOrigin();
		const tf2::Vector3 x_out  = (1.0 - alpha_) * x_prev + alpha_ * x_new;

		// Blend rotation (slerp with hemisphere fix)
		tf2::Quaternion q_prev = state_.getRotation();
		tf2::Quaternion q_new  = sample.getRotation();
		if (q_prev.dot(q_new) < 0.0) {
			q_new = -q_new; // shortest path
		}
		tf2::Quaternion q_out = q_prev.slerp(q_new, alpha_);
		q_out.normalize();

		state_.setOrigin(x_out);
		state_.setRotation(q_out);
		return state_;
	}

  private:
	static double clampAlpha(double a) {
		const double eps = 1e-9;
		return std::min(1.0, std::max(eps, a));
	}

	double         alpha_;
	bool           initialized_;
	size_t         samples_{0};
	tf2::Transform state_;
};
