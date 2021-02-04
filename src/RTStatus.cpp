#include "RTStatus.hpp"
#include "Conn.hpp"
#include "Decoder.hpp"
#include "Vec3.hpp"
#include <array>
#include <ctime>
#include <utility>

#include <iostream>

static const std::array<const rapidjson::GenericPointer<rapidjson::Value>, 7>
    p = {rapidjson::Pointer("/data/timestamp"),
         rapidjson::Pointer("/data/flying"),
         rapidjson::Pointer("/data/battery"),
         rapidjson::Pointer("/data/speed"),
         rapidjson::Pointer("/data/position/0"),
         rapidjson::Pointer("/data/position/1"),
         rapidjson::Pointer("/data/position/2")};

RTStatus::RTStatus(std::string name)
    : flying_{false}, name_(std::move(name)), speed_{0}, battery_{0}, pos_(0),
      d_(rapidjson::kObjectType), w_(sb_) {

	auto &allocator = d_.GetAllocator();

	const std::string type = Decoder::cmd_to_cstr(cmd_t::pulse);
	d_.AddMember("type", type, allocator);

	rapidjson::Value data_(rapidjson::kObjectType);
	data_.AddMember("name", name_, allocator);

	d_.AddMember("data", data_, allocator);
}

conn::msg_t RTStatus::encode() {
	sb_.Clear();
	w_.Reset(sb_);

	auto &allocator = d_.GetAllocator();
	p[0].Set(d_, std::time(nullptr), allocator);
	p[1].Set(d_, flying_, allocator);
	p[2].Set(d_, battery_, allocator);
	p[3].Set(d_, speed_, allocator);
	p[4].Set(d_, pos_.x(), allocator);
	p[5].Set(d_, pos_.y(), allocator);
	p[6].Set(d_, pos_.z(), allocator);

	d_.Accept(w_);

	return std::make_pair(sb_.GetString(), sb_.GetSize());
}

void RTStatus::update(std::float_t battery, const Vec4 &pos) {
	if (!flying_) {
		return;
	}

	battery_ = battery;
	/* 10 is the tickrate in <framework> in config.xml */
	speed_ = Vec3::norm(Vec3::sub(pos, pos_)) / 10;
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
