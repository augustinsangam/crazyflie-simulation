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
	idle,                // 0
	take_off,            // 1
	land,                // 2
	auto_pilot,          // 3
	exploration_dodge,   // 4
	stabilize,           // 5
	return_to_base,      // 6
	return_to_base_dodge // 7
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
	Dodger dodge_type_ = unset;

	bool dodging_ = false;
	bool avoiding_ = false;
	bool is_returning_to_base_ = false;

	NextMove last_move_{Vec4(0), true, 0};
	uint16_t ticks_since_last_dodge_counter_ = 0;

	Vec4 initial_pos_ = Vec4(0);
	Vec4 desired_position_ = Vec4(0);
	float_t desired_angle_ = 0;

	/**
	 * @brief Takes care of the operations to do depending on the battery level.
	 * If the battery is under BATTERY_THRESHOLD_RTB, the drone automatically
	 * begins to return to its take off point. If the battery is under
	 * BATTERY_THRESHOLD_EMERGENCY, the drone lands immediately to prevent him
	 * from falling on the ground.
	 *
	 * @param battery_level the current battery level
	 */
	void doBatteryChecks(const double &battery_level);

	/**
	 * @brief Checks the time since when we started avoiding an obstacle which
	 * is in our way to the take off point.
	 *
	 * @return true if we have been dodging for enough time
	 * @return false if we have not
	 */
	bool dodgeTimeExpired();

	/**
	 * @brief Uses the multiranger data to find out in which direction of
	 * rotation we should rotate to dodge an obstacle. Essentially f there is a
	 * close wall on our left, we should rotate clockwise. If there is a close
	 * wall on our left, we should rotate counter clockwise. This improves the
	 * total exploration area of the drones. This function also stores the new
	 * target orientation in desired_angle_.
	 *
	 * @param sd multiranger data
	 * @param currentYaw current orientation
	 * @return float_t the rotation we should apply on the drone (+ or - PI/12)
	 */
	float_t getDodgeRotation(const SensorData *sd, const float_t &currentYaw);

	/**
	 * @brief Checks if we reached the target stabilization position. Uses a
	 * tolerance value STABILIZE_POS_PRECISION to stabilize faster.
	 *
	 * @param pos the current position
	 * @return true if we are in a certain zone around the target position
	 * @return false if not
	 */
	bool isStabilized(const Vec4 &pos);

	/**
	 * @brief Stores the required parameters before stabilizing the drone.
	 *
	 * @param position The position to stabilize to
	 * @param orientation The orientation to stabilize to
	 * @param next_state The state to apply once we are stabilized.
	 */
	void setupStabilization(Vec4 position, float_t orientation,
	                        State next_state);

public:
	explicit Brain(uint16_t id) : id_(id){};

	/**
	 * @brief Finds and returns the next move the drone should do.
	 *
	 * This function takes in input the different sensors it has (the FlowDeck
	 * data, the multiranger data, and the battery level), and decides what the
	 * drone should do with these input.
	 *
	 * 7 states are defined :
	 *    - idle : do nothing and stay where you are
	 *    - take_off : go up until you reach your cruise altitude, then start
	 * exploring by going in auto_pilot
	 *    - land : go down until you reach the ground, then go idle.
	 *    - auto_pilot : move forward until you see an obstacle too close on
	 * your side or front. If you are in front of an obstacle and you are still
	 * exploring, go exploration_dodge. If you are in front of an obstacle and
	 * you are trying to return to your base, go return_to_base_dodge
	 *    - exploration_dodge : rotate until no obstacle is on your side or
	 * front, and return to auto_pilot
	 *    - stabilize : do nothing until you reached the position you are
	 * supposed to be.
	 *    - return_to_base : Rotate to the direction of the base. Then go
	 * auto_pilot with the flag returning_to_base_ as true
	 *    - return_to_base_dodge : rotate and move to the side in order to avoid
	 * an obstacle on your way to the base. When nothing is in front of you
	 * anymore, go return_to_base.
	 *
	 * x,y,z and yaw are
	 * the current position and orientation of the drone at the time the
	 * function is called
	 *
	 * @param cd current flowdeck data
	 * @param sd current multiranger data
	 * @param battery_level current battery level
	 * @return std::optional<NextMove> struct containing the next instruction.
	 * If nullopt, that means the last instruction was not finished.
	 */
	std::optional<NextMove> computeNextMove(const CameraData *cd,
	                                        const SensorData *sd,
	                                        const double &battery_level);

	/**
	 * @brief Get the state of the brain
	 *
	 * @return state
	 */
	State getState() { return state_; };

	/**
	 * @brief Set the State
	 *
	 * @param newState
	 */
	void setState(State newState) { state_ = newState; };

	/**
	 * @brief Checks if the drone is returning to its base
	 *
	 * @return true if yes
	 * @return false if no
	 */
	bool isReturningToBase() const { return is_returning_to_base_; }

	/**
	 * @brief Set the Initial Position
	 *
	 * @param pos initial position
	 */
	void setInitialPosition(Vec4 pos) { initial_pos_ = pos; }
};

} // namespace brain

#endif /* BRAIN_HPP */
