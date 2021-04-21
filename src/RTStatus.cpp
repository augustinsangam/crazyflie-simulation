#include "RTStatus.hpp"
#include "Brain.hpp"
#include "SensorData.hpp"
#include "Vec3.hpp"
#include <array>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <spdlog/common.h>
#include <spdlog/spdlog.h>
#include <sstream>
#include <tao/json/pointer.hpp>
#include <utility>

#include <tao/json.hpp>

/**
 * @brief Construct a new RTStatus::RTStatus object
 *
 * @param name name to identify the drone
 */
RTStatus::RTStatus(std::string name)
    : flying_{false}, name_(std::move(name)), speed_{0}, battery_{100},
      sensor_data_(), pos_(0), yaw_(0) {}
/**
 * @brief Encodes the current status of the drone as json
 *
 * @return std::string serialized pulse object
 */
std::string RTStatus::encode() {

	float yaw_rounded = yaw_;

	const tao::json::value pulse = {
	    {"type", "pulse"},
	    {"data",
	     {{"timestamp", std::time(nullptr)},
	      {"name", name_},
	      {"flying", flying_},
	      {"battery", battery_},
	      {"speed", speed_},
	      {"position", tao::json::value::array({pos_.x(), pos_.y(), pos_.z()})},
	      {"yaw", yaw_rounded},
	      {"ranges",
	       tao::json::value::array({sensor_data_.front, sensor_data_.left,
	                                sensor_data_.back, sensor_data_.right})},
	      {"state", pulse_state},
	      {"ledOn", false},
	      {"real", false}}}};

	return tao::json::to_string(pulse).append("\n");
}

/**
 * @brief Updates the status of the drone (battery, position, orientation,
 * speed, brain state). Sets the RTStatus::state_ approprietly depending on the
 * brain state
 *
 * @param battery battery level
 * @param pos absolute position gotten from the flowdeck
 * @param yaw absolute orientation gotten from the flowdeck
 * @param sd multiranger sensors data
 * @param brain_state current state of the brain class
 * @param brain_returning_to_base boolean to indicate if the drone is currently
 * returning to its base
 */
void RTStatus::update(std::float_t battery, const Vec4 &pos, const float_t &yaw,
                      const SensorData &sd, const brain::State &brain_state,
                      const bool &brain_returning_to_base) {
	battery_ = battery * 100;

	if (!flying_) {
		return;
	}

	/* 8 is the tickrate in <framework> in config.xml */
	speed_ = Vec3::norm(Vec3::sub(pos, pos_)) / 8;
	pos_ = pos;
	sensor_data_ = sd;
	yaw_ = yaw;

	switch (brain_state) {
	case brain::State::exploration_dodge:
		pulse_state = pulse_states[PulseIndex::exploring];
		break;

	case brain::State::auto_pilot:
		pulse_state = brain_returning_to_base
		                  ? pulse_states[PulseIndex::returningToBase]
		                  : pulse_states[PulseIndex::exploring];
		break;

	case brain::State::return_to_base:
	case brain::State::return_to_base_dodge:
		pulse_state = pulse_states[PulseIndex::returningToBase];
		break;

	case brain::State::take_off:
		pulse_state = pulse_states[PulseIndex::takingOff];
		break;

	case brain::State::land:
		pulse_state = pulse_states[PulseIndex::landing];
		break;

	case brain::State::idle:
		pulse_state = pulse_states[PulseIndex::onTheGround];
		if (flying_) {
			disable();
		}
		break;

	case brain::State::stabilize:
		pulse_state = brain_returning_to_base
		                  ? pulse_states[PulseIndex::returningToBase]
		                  : pulse_states[PulseIndex::exploring];
		break;
	}
}

/**
 * @brief Set the drone as flying
 *
 */
void RTStatus::enable() { flying_ = true; }

/**
 * @brief Set the drone as not flying
 *
 */
void RTStatus::disable() { flying_ = false; }
