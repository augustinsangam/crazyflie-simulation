#ifndef BRAIN_HPP
#define BRAIN_HPP

#include "CameraData.hpp"
#include "SensorData.hpp"
#include "Vec4.hpp"
#include <cmath>
#include <iostream>
#include <optional>

namespace brain {

struct NextMove {
	Vec4 coords;
	bool relative;
	float_t yaw;
};

enum State {
	idle,
	take_off,
	land,
	do_squares,
	auto_pilot,
	dodge,
	orient,
	stabilize
};

class Brain {

	State state_{};
	State afterStab_{};
	bool dodging_;

	const CameraData *cd_{};
	const SensorData *sd_{};
	NextMove lastMove_{Vec4(0), true, 0};

	std::array<Vec4, 4> squareMoves_ = {Vec4(0.1F, 0, 0), Vec4(0, 0.1F, 0),
	                                    Vec4(-0.1F, 0, 0), Vec4(0, -0.1F, 0)};
	int counter_ = 0;
	int squareSize_ = 50;
	float_t desiredAngle_ = 0;

	void land();
	void takeOff();
	void doSquares();

public:
	Brain() = default;

	void setState(State newState) { state_ = newState; };

	State getState() { return state_; };

	std::optional<NextMove> computeNextMove(const CameraData *cd,
	                                        const SensorData *sd);
};

} // namespace brain

#endif /* BRAIN_HPP */
