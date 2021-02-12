#ifndef SENSORDATA_HPP
#define SENSORDATA_HPP

#include <bits/stdint-uintn.h>
class SensorData {
public:
	uint16_t left;
	uint16_t right;
	uint16_t front;
	uint16_t back;
	uint16_t up;
};

#endif
