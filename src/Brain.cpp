#include "Brain.hpp"
#include "Vec4.hpp"
#include <argos3/core/utility/math/angles.h>
#include <bits/stdint-uintn.h>
#include <spdlog/spdlog.h>
#include <string>

namespace brain {

std::optional<NextMove> Brain::computeNextMove(const CameraData *cd,
                                               const SensorData *sd) {
	/*

	            front
	            [y]
	            |
	            |
	    left    L______ [x] right



	            back

	*/

	cd_ = cd;
	sd_ = sd;
	float_t x = cd->delta_x;
	float_t y = cd->delta_y;
	float_t z = cd->z;

	uint16_t sensor_wall_distance_thresh = 30; // mm ?
	double PI = 3.1415927;
	spdlog::info(
	    "f: " + std::to_string(sd->front) + " l: " + std::to_string(sd->left) +
	    " b: " + std::to_string(sd->back) + " r: " + std::to_string(sd->right));

	/**
	 * @brief State machine
	 * For now : idle -> take_off -> do_squares -> land -> idle
	 */
	NextMove nm{Vec4(0), true, desiredAngle_};
	bool overwrite = false;

	switch (state_) {

	case State::idle:
		spdlog::info("Idle");
		nm = {Vec4(0), true, desiredAngle_};
		// state_ = dodge;
		break;

	case State::take_off:
		if (cd->z >= 0.5) {
			state_ = State::auto_pilot;
		}
		spdlog::info("Take Off");
		nm = {Vec4(0, 0, 0.5F), true, desiredAngle_};
		break;

	case State::orient:
		nm = {Vec4(0), true, static_cast<float_t>(PI / 2)};
		state_ = auto_pilot;
		break;

	case State::stabilize:
		spdlog::info("Stabilize");
		if (++counter_ > 50) {
			nm = {Vec4(x, y, z), false, cd->yaw};
			state_ = afterStab_;
			counter_ = 0;
		}
		// spdlog::info(cd->yaw << " -> " << desiredAngle_);
		// if (desiredAngle_ - 0.001 < cd->yaw &&
		//     cd->yaw < desiredAngle_ + 0.001) {
		// 	state_ = dodge;
		// }
		break;

	case State::dodge:
		spdlog::info("dodge");
		if (!dodging_) {
			dodging_ = true;
			desiredAngle_ = cd->yaw + PI / 12.0F;
			spdlog::info("desiredAngle_ " + std::to_string(desiredAngle_));
			nm = {Vec4(0), true, static_cast<float_t>(PI / 12.0F)};
			counter_ = 0;
			break;
		}
		if (++counter_ < 30) {
			nm = {Vec4(x, y, z), false, cd->yaw};
			dodging_ = true;
			break;
		}
		dodging_ = false;
		if (sd->right < sensor_wall_distance_thresh || sd->front > 100) {
			state_ = auto_pilot;
			dodging_ = false;
			break;
		}
		break;

	case State::land:
		if (cd->z - 0.1 < 0.1F) {
			state_ = State::idle;
		}
		spdlog::info("Land");
		nm = {Vec4(0, 0, 0), false, desiredAngle_};
		break;

	case State::auto_pilot:
		if (sd->front < sensor_wall_distance_thresh) {
			// this will stop the drone
			nm = {Vec4(x, y, z), false, cd->yaw};
			// afterStab_ = dodge;
			state_ = dodge;
			break;
		}
		spdlog::info("auto_pilot");
		nm = {Vec4(0, -0.05F, 0), true, 0};
		// if (cd->delta_x > -0.5 && cd->delta_y > -0.5) {
		overwrite = true;
		// }

		break;

		/* case State::do_squares:
		    Vec4 nextMove(0);
		    ++counter_;
		    if (counter_ < 1 * squareSize_) {
		        nextMove = squareMoves_[0];
		    } else if (counter_ < 2 * squareSize_) {
		        nextMove = squareMoves_[1];
		    } else if (counter_ < 3 * squareSize_) {
		        nextMove = squareMoves_[2];
		    } else if (counter_ < 4 * squareSize_) {
		        nextMove = squareMoves_[3];
		    } else {
		        counter_ = 0;
		        state_ = State::land;
		        return {Vec4(0, 0, 0), false};
		    }
		    return {nextMove, true};*/
	}
	if (nm.coords == lastMove_.coords && nm.relative == lastMove_.relative &&
	    nm.yaw == lastMove_.yaw && !overwrite) {
		return std::nullopt;
	}
	lastMove_ = nm;
	return nm;
}

} // namespace brain
