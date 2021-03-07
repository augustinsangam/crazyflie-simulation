#ifndef BRAIN_HPP
#define BRAIN_HPP

#include "CameraData.hpp"
#include "SensorData.hpp"
#include "Vec4.hpp"
#include <bits/stdint-uintn.h>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>

namespace brain {

struct NextMove {
	Vec4 coords;
	bool relative;
	float_t yaw;
};

enum State { idle, take_off, land, auto_pilot, dodge, stabilize };

class Brain {

	State state_{};
	State afterStab_{};
	bool dodging_ = false;
	uint16_t id_;
	Vec4 initial_pos_ = Vec4(0);

	const CameraData *cd_{};
	const SensorData *sd_{};
	NextMove lastMove_{Vec4(0), true, 0};

	std::array<Vec4, 4> squareMoves_ = {Vec4(0.1F, 0, 0), Vec4(0, 0.1F, 0),
	                                    Vec4(-0.1F, 0, 0), Vec4(0, -0.1F, 0)};
	int counter_ = 0;
	int squareSize_ = 50;
	float_t desiredAngle_ = 0;
	Vec4 desiredPosition_ = Vec4(0);

	void land();
	void takeOff();
	void doSquares();
	void setupStabilization(Vec4 position, float_t orientation,
	                        State next_state);

public:
	explicit Brain(uint16_t id) : id_(id){};

	void setState(State newState) { state_ = newState; };

	State getState() { return state_; };

	std::optional<NextMove> computeNextMove(const CameraData *cd,
	                                        const SensorData *sd);

	void setInitialPosition(Vec4 pos) { initial_pos_ = pos; }
};

} // namespace brain

#endif /* BRAIN_HPP */
