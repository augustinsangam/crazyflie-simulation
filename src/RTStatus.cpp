#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#define RAPIDJSON_HAS_STDSTRING 1
#include "RTStatus.hpp"

namespace rj = rapidjson;

RTStatus::RTStatus()
    : name(""), batteryLevel(0), posX(0), posY(0), posZ(0), speed(0),
      isOn(false), sb(), writer(sb) {}

RTStatus::RTStatus(const std::string &name)
    : name(name), batteryLevel(0), posX(0), posY(0), posZ(0), speed(0),
      isOn(false), sb(), writer(sb) {
	enable();
}

std::string RTStatus::encode() {
	this->sb.Clear();
	this->writer.Reset(sb);

	writer.StartObject();
	writer.String("type");
	writer.String("robot_update");
	writer.String("data");
	writer.StartObject();
	writer.String("name");
	writer.String(name.c_str());
	writer.String("speed");
	writer.Double(speed);
	writer.String("batteryPercentage");
	writer.Double(batteryLevel * 100);
	writer.String("localisation");
	writer.StartObject();
	writer.String("x");
	writer.Double(posX);
	writer.String("y");
	writer.Double(posY);
	writer.String("z");
	writer.Double(posZ);
	writer.EndObject();
	writer.String("lastUpdate");
	writer.Int64(std::time(nullptr));
	writer.String("isOn");
	writer.Bool(isOn);
	writer.EndObject();

	return sb.GetString();
}

void RTStatus::update(argos::Real batteryLevelU, argos::Real posXU,
                      argos::Real posYU, argos::Real posZU) {
	if (!isOn) {
		return;
	}
	this->batteryLevel = batteryLevelU;
	double dist = sqrt(pow(posXU - posX, 2) + pow(posYU - posY, 2) +
	                   pow(posZU - posZ, 2));
	/* 10 is the tickrate in <framework> in config.xml */
	this->speed = dist / (double)10;
	this->posX = posXU;
	this->posY = posYU;
	this->posZ = posZU;
};

void RTStatus::enable() { isOn = true; }

void RTStatus::disable() { isOn = false; }

void RTStatus::print() const {
	std::cout << "Updated data : " << std::endl
	          << "battery_level: " << this->batteryLevel << std::endl
	          << "posX: " << this->posX << std::endl
	          << "posY: " << this->posY << std::endl
	          << "posZ: " << this->posZ << std::endl;
}
