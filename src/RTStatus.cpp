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
    : flying_{false}, name_(std::move(name)), speed_{0}, battery_{100},
      pos_(0) {}

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

void RTStatus::update(std::float_t battery, const Vec4 &pos) {
	if (!flying_) {
		return;
	}

	battery_ = battery * 100;
	/* 8 is the tickrate in <framework> in config.xml */
	speed_ = Vec3::norm(Vec3::sub(pos, pos_)) / 8;
	pos_ = pos;
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
