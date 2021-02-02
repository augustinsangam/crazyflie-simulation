#include "RTStatus.hpp"
#include "Vec3.hpp"
#include <array>
#include <ctime>
#include <utility>

#include <iostream>

static const std::array<const rapidjson::GenericPointer<rapidjson::Value>, 6>
    p = {rapidjson::Pointer("/data/timestamp"),
         rapidjson::Pointer("/data/battery"),
         rapidjson::Pointer("/data/speed"),
         rapidjson::Pointer("/data/position/0"),
         rapidjson::Pointer("/data/position/1"),
         rapidjson::Pointer("/data/position/2")};

RTStatus::RTStatus(std::string name)
    : flying_{false}, name_(std::move(name)), speed_{0}, battery_{0}, pos_(0),
      d_(rapidjson::kObjectType), w_(sb_) {

	enable();

	auto &allocator = d_.GetAllocator();

	d_.AddMember("type", "robot_update", allocator);

	rapidjson::Value data_(rapidjson::kObjectType);
	data_.AddMember("name", name_, allocator);
	data_.AddMember("is_on", flying_, allocator);

	d_.AddMember("data", data_, allocator);
}

std::string RTStatus::encode() {
	sb_.Clear();
	w_.Reset(sb_);

	auto &allocator = d_.GetAllocator();
	p[0].Set(d_, std::time(nullptr), allocator);
	p[1].Set(d_, battery_, allocator);
	p[2].Set(d_, speed_, allocator);
	p[3].Set(d_, pos_.x(), allocator);
	p[4].Set(d_, pos_.y(), allocator);
	p[5].Set(d_, pos_.z(), allocator);

	d_.Accept(w_);

	return sb_.GetString();
}

void RTStatus::update(std::float_t battery, const Vec4 &pos) {
	if (!flying_) {
		return;
	}

	battery_ = battery;
	/* 10 is the tickrate in <framework> in config.xml */
	speed_ = Vec3::norm(Vec3::sub(pos, pos_)) / 10;
	pos_ = pos;
};

void RTStatus::enable() { flying_ = true; }

void RTStatus::disable() { flying_ = false; }

void RTStatus::print() const {
	std::cout << "Updated data : " << std::endl
	          << "battery_level: " << battery_ << std::endl
	          << "pos: " << pos_.x() << " " << pos_.y() << " " << pos_.z()
	          << std::endl;
}
