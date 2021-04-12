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
#include <spdlog/spdlog.h>

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
	auto_pilot,
	dodge,
	stabilize,
	return_to_base,
	avoid_obstacle
};

enum Dodger { clockwise, counter_clockwise, unset };

/**
 * @brief Brain : the class that decides what the drone should do next depending
 * on where it is, what it sees, and what it receives from the station.
 *
 */
class Brain {

	float_t PI = 3.141593F;
	uint16_t id_;
	static constexpr double battery_threshold_ = 0.60;
	State state_{};
	State afterStab_{};
	bool dodging_ = false;
	bool avoiding_ = false;
	Vec4 initial_pos_ = Vec4(0);
	bool is_returning_to_base_ = false;
	Vec4 auto_pilot_target_position_ = Vec4(0);
	Dodger dodge_type_ = unset;

	const CameraData *cd_{};
	const SensorData *sd_{};
	NextMove lastMove_{Vec4(0), true, 0};

	int counter_ = 0;
	float_t desiredAngle_ = 0;
	Vec4 desiredPosition_ = Vec4(0);

	void land();
	void takeOff();
	void doSquares();
	void setupStabilization(Vec4 position, float_t orientation,
	                        State next_state);
	float computeDirectionToBase(const Vec4 &pos) const;

public:
	explicit Brain(uint16_t id) : id_(id){};

	/**
	 * @brief Set the State
	 *
	 * @param newState
	 */
	void setState(State newState) { state_ = newState; };

	/**
	 * @brief Get the state of the brain
	 *
	 * @return state
	 */
	State getState() { return state_; };

	/**
	 * @brief Set the Initial Position
	 *
	 * @param pos initial position
	 */
	void setInitialPosition(Vec4 pos) { initial_pos_ = pos; }

	std::optional<NextMove> computeNextMove(const CameraData *cd,
	                                        const SensorData *sd,
	                                        const double &battery_level);
	float_t getDodgeRotation(const SensorData *sd, const float_t &currentYaw);

	bool isReturningToBase() { return is_returning_to_base_; }
};

} // namespace brain

#endif /* BRAIN_HPP */
