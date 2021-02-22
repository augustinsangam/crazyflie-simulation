#ifndef BRAIN_HPP
#define BRAIN_HPP

#include "CameraData.hpp"
#include "SensorData.hpp"
#include "Vec4.hpp"
#include <iostream>

namespace brain {

enum State { idle, take_off, land, do_squares };

class Brain {

	State state_{};

	const CameraData *cd_{};
	const SensorData *sd_{};

	void land();
	void takeOff();
	void doSquares();

public:
	Brain() = default;

	void setState(State newState) { state_ = newState; };

	Vec4 computeNextMove(const CameraData *cd, const SensorData *sd);
};

}; // namespace brain

#endif /* BRAIN_HPP */
