#include "Brain.hpp"

std::map<std::string, RTStatus> Brain::map_;

void Brain::registerDrone(const std::string &droneName) {
	for (auto &it : Brain::map_) {
		if (it.first == droneName) {
			std::cout << "Drone " << droneName << " is already registered"
			          << std::endl;
			return;
		}
	}
	RTStatus rtStatus(droneName);
	Brain::map_.insert(std::pair<std::string, RTStatus>(droneName, rtStatus));
}

RTStatus Brain::getRtStatus(const std::string &droneName) {
	auto it = Brain::map_.find(droneName);
	if (it != Brain::map_.end()) {
		return it->second;
	}
	std::cout << "Warning: " << droneName << " was not found in the map"
	          << std::endl;
	return RTStatus(droneName);
}

void Brain::updatePos(const std::string &droneName, std::float_t posX,
                      std::float_t posY, std::float_t posZ) {
	RTStatus rts = Brain::getRtStatus(droneName);
	rts.setPosition(Vec4(static_cast<std::float_t>(posX),
	                     static_cast<std::float_t>(posY),
	                     static_cast<std::float_t>(posZ)));
}

void Brain::updateSensors(const std::string &droneName, uint16_t left,
                          uint16_t right, uint16_t front, uint16_t back,
                          uint16_t up) {
	RTStatus rts = Brain::getRtStatus(droneName);
	rts.setSensorData(left, right, front, back, up);
}

void Brain::updateCamera(const std::string &droneName, uint16_t delta_x_l,
                         uint16_t delta_x_h, uint16_t delta_y_l,
                         uint16_t delta_y_h) {
	RTStatus rts = Brain::getRtStatus(droneName);
	rts.setCameraData(delta_x_l, delta_x_h, delta_y_l, delta_y_h);
}

void Brain::updateBattery(const std::string &droneName,
                          std::float_t batteryCharge) {
	RTStatus rts = Brain::getRtStatus(droneName);
	rts.setBattery(batteryCharge);
}

cmd_t Brain::nextInstruction() { return unknown; }
