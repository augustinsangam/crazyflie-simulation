#ifndef BRAIN_HPP
#define BRAIN_HPP

#include "Decoder.hpp"
#include "RTStatus.hpp"
#include <iostream>
#include <map>

class Brain {
private:
	static std::map<std::string, RTStatus> map_;
	static RTStatus getRtStatus(const std::string &droneName);

public:
	Brain() = default;
	static void registerDrone(const std::string &droneName);
	void updatePos(const std::string &droneName, std::float_t posX,
	               std::float_t posY, std::float_t posZ);
	void updateBattery(const std::string &droneName,
	                   std::float_t batteryCharge);
	void updateCamera(const std::string &droneName, uint16_t delta_x_l,
	                  uint16_t delta_x_h, uint16_t delta_y_l,
	                  uint16_t delta_y_h);
	void updateSensors(const std::string &droneName, uint16_t left,
	                   uint16_t right, uint16_t front, uint16_t back,
	                   uint16_t up);
	cmd_t nextInstruction();
	std::string encode();
};
#endif
