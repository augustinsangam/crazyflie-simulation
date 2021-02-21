#ifndef SENSORDATA_HPP
#define SENSORDATA_HPP

#include <cstdint>

struct SensorData {
public:
	std::uint16_t front, left, back, right, up;
};

#endif /* SENSORDATA_HPP */
