#ifndef BRAIN_HPP
#define BRAIN_HPP

#include "CameraData.hpp"
#include "Constants.hpp"
#include "MathUtils.hpp"
#include "SensorData.hpp"
#include "Vec4.hpp"
#include <argos3/core/utility/math/angles.h>
#include <bits/stdint-uintn.h>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>
#include <sys/types.h>

namespace brain {

struct NextMove {
	Vec4 coords;
	bool relative;
	float_t yaw;
};

enum State {
	idle,           // 0
	take_off,       // 1
	land,           // 2
	auto_pilot,     // 3
	dodge,          // 4
	stabilize,      // 5
	return_to_base, // 6
	avoid_obstacle  // 7
};

enum Dodger { clockwise, counter_clockwise, unset };

/**
 * @brief Brain : the class that decides what the drone should do next depending
 * on where it is, what it sees, and what it receives from the station.
 *
 */
class Brain {

	uint16_t id_;
	State state_{};
	State next_state_{};
	bool dodging_ = false;
	bool avoiding_ = false;
	Vec4 initial_pos_ = Vec4(0);
	bool is_returning_to_base_ = false;
	Vec4 auto_pilot_target_position_ = Vec4(0);
	Dodger dodge_type_ = unset;
	const CameraData *cd_{};
	const SensorData *sd_{};
	NextMove last_move_{Vec4(0), true, 0};

	int counter_ = 0;
	Vec4 desired_position_ = Vec4(0);
	float_t desired_angle_ = 0;

	void setupStabilization(Vec4 position, float_t orientation,
	                        State next_state);

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

	void doBatteryChecks(const double &battery_level);

	bool isStabilized(const Vec4 &pos);

	bool isReturningToBase() const { return is_returning_to_base_; }
};

} // namespace brain

#endif /* BRAIN_HPP */
