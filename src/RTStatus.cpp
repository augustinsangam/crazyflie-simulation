#include "RTStatus.hpp"
#include "Decoder.hpp"
#include "Vec3.hpp"
#include <array>
#include <ctime>
#include <iostream>
#include <utility>

#include <rapidjson/encodings.h>
#include <rapidjson/writer.h>

class StringHolder {
private:
	std::string *s_;

public:
	using Ch = rapidjson::UTF8<>::Ch;

	explicit StringHolder(std::string *s) : s_(s) { s_->reserve(4096); }
	std::size_t Size() const { return s_->length(); }
	void Put(char c) { s_->push_back(c); }
	void Clear() { s_->clear(); }
	void Flush() {}
};
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
	std::string s;
	StringHolder sh(&s);
	rapidjson::Writer<StringHolder> w(sh);

	w.StartObject();

	w.String("type");
	w.String("pulse");

	w.String("data");
	w.StartObject();

	w.String("timestamp");
	w.Int64(std::time(nullptr));

	w.String("name");
	w.String(name_);

	w.String("flying");
	w.Bool(flying_);

	w.String("battery");
	w.Double(battery_);

	w.String("speed");
	w.Double(speed_);

	w.String("position");
	w.StartArray();
	w.Double(pos_.x());
	w.Double(pos_.y());
	w.Double(pos_.z());
	w.EndArray();

	w.String("ledOn");
	w.Bool(false);

	w.String("real");
	w.Bool(false);

	w.EndObject();

	w.EndObject();

	return s;
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
