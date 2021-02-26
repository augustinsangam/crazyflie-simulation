#include "Brain.hpp"
#include "Vec4.hpp"

namespace brain {

NextMove Brain::computeNextMove(const CameraData *cd, const SensorData *sd) {
	cd_ = cd;
	sd_ = sd;
	float_t x = cd->delta_x;
	float_t y = cd->delta_y;
	float_t z = cd->z;

	/**
	 * @brief State machine
	 * For now : idle -> take_off -> do_squares -> land -> idle
	 */
	switch (state_) {

	case State::idle:
		std::cout << "Idle" << std::endl;
		return {Vec4(x, y, z), false};

	case State::take_off:
		if (cd->z >= 0.85) {
			state_ = State::do_squares;
		}
		std::cout << "Take Off" << std::endl;
		return {Vec4(x, y, 0.91F), false};

	case State::land:
		if (cd->z - 0.1 < 0.1F) {
			state_ = State::idle;
		}
		std::cout << "Land" << std::endl;
		return {Vec4(0, 0, 0), false};

	case State::do_squares:
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
		return {nextMove, true};

		// default:
		// 	return {Vec4(0, 0, 0), true};
	}
}

} // namespace brain
