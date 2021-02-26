#include "Brain.hpp"
#include "Vec4.hpp"
#include <bits/stdint-uintn.h>

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

	uint16_t sensor_wall_distance_thresh = 15; // mm ?
	double PI = 3.1415927;
	std::cout << "front: " << sd->front << " left: " << sd->left
	          << " back: " << sd->back << " right: " << sd->right << std::endl;

	/**
	 * @brief State machine
	 * For now : idle -> take_off -> do_squares -> land -> idle
	 */
	NextMove nm{Vec4(0), true, 0};
	bool overwrite = false;

	switch (state_) {

	case State::idle:
		std::cout << "Idle" << std::endl;
		nm = {Vec4(x, y, z), false, 0};
		break;

	case State::take_off:
		if (cd->z >= 0.5) {
			state_ = State::auto_pilot;
		}
		std::cout << "Take Off" << std::endl;
		nm = {Vec4(0, 0, 0.5F), true, 0};
		break;

	case State::orient:
		nm = {Vec4(0), true, static_cast<float_t>(PI / 2)};
		state_ = auto_pilot;
		break;

	case State::dodge:
		std::cout << "dodge" << std::endl;
		if (sd->right > sensor_wall_distance_thresh) {
			nm = {Vec4(0), true, static_cast<float_t>(PI / 12)};
			overwrite = true;
		}
		state_ = auto_pilot;
		break;

	case State::land:
		if (cd->z - 0.1 < 0.1F) {
			state_ = State::idle;
		}
		std::cout << "Land" << std::endl;
		nm = {Vec4(0, 0, 0), false, 0};
		break;

	case State::auto_pilot:
		std::cout << "auto_pilot" << std::endl;
		if (sd->front < sensor_wall_distance_thresh) {
			state_ = dodge;
		}
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
	    !overwrite) {
		return std::nullopt;
	}
	lastMove_ = nm;
	return nm;
}

} // namespace brain
