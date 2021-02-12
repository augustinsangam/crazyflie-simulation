#include "RTStatus.hpp"
#include "Conn.hpp"
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

RTStatus::RTStatus(std::string name)
    : flying_{false}, name_(std::move(name)), speed_{0}, battery_{0}, pos_(0) {}

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

	w.EndObject();

	w.EndObject();

	return s;
}

void RTStatus::enable() { flying_ = true; }

void RTStatus::disable() { flying_ = false; }

void RTStatus::print() const {
	std::cout << "Updated data : " << std::endl
	          << "battery_level: " << battery_ << std::endl
	          << "pos: " << pos_.x() << " " << pos_.y() << " " << pos_.z()
	          << std::endl;
}

std::string RTStatus::getName() {
	std::string shortName;
	for (size_t i = 0; i < 5; i++) {
		shortName += name_[i];
	}
	return shortName;
}

void RTStatus::setSensorData(uint16_t left, uint16_t right, uint16_t front,
                             uint16_t back, uint16_t up) {
	sensors_.left = left;
	sensors_.right = right;
	sensors_.front = front;
	sensors_.back = back;
	sensors_.up = up;
}

void RTStatus::setCameraData(uint16_t delta_x_l, uint16_t delta_x_h,
                             uint16_t delta_y_l, uint16_t delta_y_h) {
	camera_.delta_x_l = delta_x_l;
	camera_.delta_x_h = delta_x_h;
	camera_.delta_y_l = delta_y_l;
	camera_.delta_y_h = delta_y_h;
}
