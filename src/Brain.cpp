#include "Brain.hpp"

namespace brain {

Vec4 Brain::computeNextMove(const CameraData *cd, const SensorData *sd) {
	cd_ = cd;
	sd_ = sd;

	switch (state_) {
	case State::idle:
		return Vec4(0, 0, 0);
	case State::take_off:
		if (cd->z > 1) {
			state_ = State::idle;
		}
		std::cout << "Take Off" << std::endl;
		return Vec4(0, 0, 0.01F);
	case State::land:
		if (cd->z < 0.02F) {
			state_ = State::idle;
		}
		std::cout << "Land" << std::endl;
		return Vec4(0, 0, -0.01F);
	default:
		return Vec4(0, 0, 0);
	}
}

} // namespace brain
