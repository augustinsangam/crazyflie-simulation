#ifndef BRAIN_HPP
#define BRAIN_HPP

#include "CameraData.hpp"
#include "SensorData.hpp"
#include "Vec4.hpp"
#include <iostream>

namespace brain {

struct NextMove {
	Vec4 coords;
	bool relative;
};

enum State { idle, take_off, land, do_squares };

class Brain {

	State state_{};

	const CameraData *cd_{};
	const SensorData *sd_{};

	std::array<Vec4, 4> squareMoves_ = {Vec4(1, 0, 0), Vec4(0, 1, 0),
	                                    Vec4(-1, 0, 0), Vec4(0, -1, 0)};
	int counter_ = 0;
	int squareSize_ = 5;

	void land();
	void takeOff();
	void doSquares();

public:
	Brain() = default;

	void setState(State newState) { state_ = newState; };

	State getState() { return state_; };

	NextMove computeNextMove(const CameraData *cd, const SensorData *sd);
};

} // namespace brain

#endif /* BRAIN_HPP */
