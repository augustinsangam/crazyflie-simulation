#include "RTStatus.hpp"
#include "Decoder.hpp"
#include "SensorData.hpp"
#include "Vec3.hpp"
#include <array>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <math.h>
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
      sensor_data_(), pos_(0) {}
/**
 * @brief Encodes the current status of the drone as json
 *
 * @return std::string serialized pulse object
 */
std::string RTStatus::encode() {

	float yaw_rounded = trunc(yaw_, 2);
	std::cout << std::setprecision(2);

	const tao::json::value pulse = {
	    {"type", "pulse"},
	    {"data",
	     {{"timestamp", std::time(nullptr)},
	      {"name", name_},
	      {"flying", flying_},
	      {"battery", trunc(battery_, 2)},
	      {"speed", trunc(speed_, 2)},
	      {"position",
	       tao::json::value::array(
	           {trunc(pos_.x(), 2), trunc(pos_.y(), 2), trunc(pos_.z(), 2)})},
	      {"yaw", yaw_rounded},
	      {"ranges",
	       tao::json::value::array({sensor_data_.front, sensor_data_.left,
	                                sensor_data_.back, sensor_data_.right})},
	      {"ledOn", false},
	      {"real", false}}}};

	// spdlog::debug(tao::json::to_string(pulse));

	// std::stringstream msg;
	// msg << std::setprecision(2);

	// msg << yaw_rounded;
	// spdlog::debug("rounded with string stream {}", msg.str());

	// tao::json::to_stream(msg, pulse);

	// std::string message = msg.str().append("\n");
	// spdlog::debug("message rounded = {}", message);

	return tao::json::to_string(pulse).append("\n");
}

/**
 * @brief Updates the status of the drone (battery, position, and speed)
 *
 * @param battery
 * @param pos
 */
void RTStatus::update(std::float_t battery, const Vec4 &pos, const float_t &yaw,
                      const SensorData &sd) {
	if (!flying_) {
		return;
	}

	battery_ = battery * 100;
	/* 8 is the tickrate in <framework> in config.xml */
	speed_ = Vec3::norm(Vec3::sub(pos, pos_)) / 8;
	pos_ = pos;
	sensor_data_ = sd;
	yaw_ = yaw;
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

float_t RTStatus::trunc(float_t val, int numDigits) {
	// int rounded_val =
	//     static_cast<int>(static_cast<double>(val) * std::pow(10, numDigits));
	// return static_cast<float_t>(rounded_val / 100.0);
	float rounded = std::roundf(val * 100) / 100;
	return rounded;
}
