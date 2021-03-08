#include "RTStatus.hpp"
#include "Decoder.hpp"
#include "Vec3.hpp"
#include <array>
#include <ctime>
#include <iostream>
#include <utility>

#include <tao/json.hpp>

/**
 * @brief Construct a new RTStatus::RTStatus object
 *
 * @param name name to identify the drone
 */
RTStatus::RTStatus(std::string name)
    : flying_{false}, name_(std::move(name)), speed_{0}, battery_{100},
      pos_(0) {}
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
	      {"battery", battery_},
	      {"speed", speed_},
	      {"position", tao::json::value::array({pos_.x(), pos_.y(), pos_.z()})},
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
void RTStatus::update(std::float_t battery, const Vec4 &pos) {
	if (!flying_) {
		return;
	}

	battery_ = battery * 100;
	/* 8 is the tickrate in <framework> in config.xml */
	speed_ = Vec3::norm(Vec3::sub(pos, pos_)) / 8;
	pos_ = pos;
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

/**
 * @brief Returns the name of the current drone
 *
 * @return std::string
 */
std::string RTStatus::getName() {
	std::string shortName;
	for (size_t i = 0; i < 5; i++) {
		shortName += name_[i];
	}
	return shortName;
}
