#ifndef SENSORDATA_HPP
#define SENSORDATA_HPP

#include <cstdint>

/**
 * @brief SensorData : struct to store the multiranger sensors. The value is
 * between 0-255 if the wall detected is reasonnably close, and 65534 if it's
 * too far away
 *
 */
struct SensorData {
public:
	std::uint16_t front, left, back, right, up;
};

#endif /* SENSORDATA_HPP */
