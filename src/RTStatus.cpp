#include "RTStatus.hpp"
#include "Decoder.hpp"
#include "SensorData.hpp"
#include "Vec3.hpp"
#include <array>
#include <cmath>
#include <ctime>
#include <iostream>
#include <math.h>
#include <utility>

#include <tao/json.hpp>

/**
 * @brief Construct a new RTStatus::RTStatus object
 *
 * @param name name to identify the drone
 */
RTStatus::RTStatus(std::string name)
    : flying_{false}, name_(std::move(name)), speed_{0}, battery_{100},
      sensor_data_(), pos_(0) {}
/**
 * @brief Encodes the current status of the drone as json
 *
 * @return std::string serialized pulse object
 */
std::string RTStatus::encode() {

	const tao::json::value pulse = {
	    {"type", "pulse"},
	    {"data",
	     {{"timestamp", std::time(nullptr)},
	      {"name", name_},
	      {"flying", flying_},
	      {"battery", trunc<float_t>(battery_, 2)},
	      {"speed", trunc<float_t>(speed_, 2)},
	      {"position", tao::json::value::array({trunc<float_t>(pos_.x(), 2),
	                                            trunc<float_t>(pos_.y(), 2),
	                                            trunc<float_t>(pos_.z(), 2)})},
	      {"ranges",
	       tao::json::value::array({sensor_data_.front, sensor_data_.left,
	                                sensor_data_.back, sensor_data_.right})},
	      {"ledOn", false},
	      {"real", false}}}};

	return tao::json::to_string(pulse);
}

/**
 * @brief Updates the status of the drone (battery, position, and speed)
 *
 * @param battery
 * @param pos
 */
void RTStatus::update(std::float_t battery, const Vec4 &pos,
                      const SensorData &sd) {
	if (!flying_) {
		return;
	}

	battery_ = battery * 100;
	/* 8 is the tickrate in <framework> in config.xml */
	speed_ = Vec3::norm(Vec3::sub(pos, pos_)) / 8;
	pos_ = pos;
	sensor_data_ = sd;
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

/**
 * @brief Debug function to display the drone status
 *
 */
void RTStatus::print() const {
	std::cout << "Updated data : " << std::endl
	          << "battery_level: " << battery_ << std::endl
	          << "pos: " << pos_.x() << " " << pos_.y() << " " << pos_.z()
	          << std::endl;
}

template <typename T> T RTStatus::trunc(T val, int numDigits) {
	// TODO
	return val;
}
