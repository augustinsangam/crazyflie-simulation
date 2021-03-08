#include "Brain.hpp"
#include "Vec4.hpp"
#include <argos3/core/utility/math/angles.h>
#include <bits/stdint-uintn.h>
#include <cstdlib>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>

namespace brain {

std::optional<NextMove> Brain::computeNextMove(const CameraData *cd,
                                               const SensorData *sd) {
	cd_ = cd;
	sd_ = sd;
	float_t x = cd->delta_x + initial_pos_.x();
	float_t y = cd->delta_y + initial_pos_.y();
	float_t z = cd->z + initial_pos_.z();

	uint16_t sensor_wall_distance_thresh = 15;
	float_t PI = 3.141593F;
	// spdlog::info(
	//     "f: " + std::to_string(sd->front) + " l: " + std::to_string(sd->left)
	//     + " b: " + std::to_string(sd->back) + " r: " +
	//     std::to_string(sd->right));
	// spdlog::info("Desired Pos -> (x: " + std::to_string(desiredPosition_.x())
	// +
	//              " y: " + std::to_string(desiredPosition_.y()) +
	//              " z: " + std::to_string(desiredPosition_.z()) +
	//              ") [tolerance=" + std::to_string(0.005) +
	//              "] yaw: " + std::to_string(desiredAngle_));
	// spdlog::info("Actual Pos -> (x: " + std::to_string(x) +
	//              " y: " + std::to_string(y) + " z: " + std::to_string(z) +
	//              ") [tolerance=" + std::to_string(0.005) +
	//              "] yaw: " + std::to_string(cd_->yaw));

	NextMove nm{Vec4(0), true, desiredAngle_};
	bool overwrite = false;

	switch (state_) {
	case State::idle:
		// spdlog::info("Idle");
		nm = {Vec4(0), true, desiredAngle_};
		break;

	case State::take_off:
		// spdlog::info("Take Off");
		if (cd->z >= 0.5F) {
			setupStabilization(Vec4(x, y, z), cd->yaw, State::auto_pilot);
			state_ = stabilize;
		}
		nm = {Vec4(0, 0, 0.5F), true, desiredAngle_};
		break;

	case State::stabilize:
		// spdlog::info("Stabilize");
		/* (debug)
		if (desiredPosition_.x() - 0.005F < cd->delta_x &&
		    cd->delta_x < desiredPosition_.x() + 0.005F) {
		    spdlog::info("x ok");
		}
		if (desiredPosition_.y() - 0.005F < cd->delta_y &&
		    cd->delta_y < desiredPosition_.y() + 0.005F) {
		    spdlog::info("y ok");
		}
		if (desiredPosition_.z() - 0.005F < z &&
		    z < desiredPosition_.z() + 0.005F) {
		    spdlog::info("z ok");
		}
		if (desiredAngle_ - 0.01F < cd->yaw &&
		    cd->yaw < desiredAngle_ + 0.01F) {
		    spdlog::info("yaw ok");
		}
		*/
		if (desiredPosition_.x() - 0.005F < cd->delta_x &&
		    cd->delta_x < desiredPosition_.x() + 0.005F &&
		    desiredPosition_.y() - 0.005F < cd->delta_y &&
		    cd->delta_y < desiredPosition_.y() + 0.005F &&
		    desiredPosition_.z() - 0.005F < z &&
		    z < desiredPosition_.z() + 0.005F &&
		    std::abs(desiredAngle_) - 0.01F < std::abs(cd->yaw) &&
		    std::abs(cd->yaw) < std::abs(desiredAngle_) + 0.01F) {
			state_ = afterStab_;
		} else {
			nm = {desiredPosition_, false, desiredAngle_};
		}
		break;

	case State::dodge:
		// spdlog::info("dodge");
		if (!dodging_) {
			// "Starting rotation"
			desiredAngle_ = cd->yaw + PI / 12.0F;
			if (desiredAngle_ > PI) {
				desiredAngle_ -= 2 * PI;
			}
			nm = {Vec4(x, y, z), false,
			      cd->yaw + static_cast<float_t>(PI / 12)};
			dodging_ = true;
		} else {
			// spdlog::info("Rotating (current " + std::to_string(cd->yaw) +
			//              " target " + std::to_string(desiredAngle_) + ")");
			if (desiredAngle_ - 0.05F < cd->yaw &&
			    cd->yaw < desiredAngle_ + 0.05F) {
				// "Reached target angle. Stopping rotation"
				overwrite = true;
				nm = {Vec4(x, y, z), false, cd->yaw};
				if (sd->front > 30) {
					// "Nothing in front of me, auto_pilot now"
					state_ = auto_pilot;
				}
				// "Something still in front of me, restarting a rotation"
				dodging_ = false;
			}
		}
		break;

	case State::land:
		// spdlog::info("Land");
		if (cd->z - 0.1F < 0.1F) {
			state_ = State::idle;
		}
		nm = {Vec4(0, 0, 0), false, desiredAngle_};
		break;

	case State::auto_pilot:
		// spdlog::info("auto_pilot");
		if (sd->front < sensor_wall_distance_thresh) {
			setupStabilization(Vec4(x, y, z), cd->yaw, State::dodge);
			state_ = stabilize;
			break;
		}
		nm = {Vec4(0, -0.03F, 0), true, desiredAngle_};
		overwrite = true;

		break;
	}
	if (nm.coords == lastMove_.coords && nm.relative == lastMove_.relative &&
	    nm.yaw == lastMove_.yaw && !overwrite) {
		return std::nullopt;
	}
	lastMove_ = nm;
	return nm;
}

void Brain::setupStabilization(Vec4 position, float_t orientation,
                               State next_state) {
	desiredPosition_ = position;
	desiredAngle_ = orientation;
	afterStab_ = next_state;
}

} // namespace brain
