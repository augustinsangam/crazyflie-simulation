#ifndef CAMERADATA_HPP
#define CAMERADATA_HPP

#include <cmath>

/**
 * @brief CameraData : struct to hold a flowdeck-like outputted data. Inspired
 * from the real flowdeck :
 * https://github.com/bitcraze/crazyflie-firmware/blob/master/src/deck/drivers/src/zranger.c
 * https://github.com/bitcraze/crazyflie-firmware/blob/master/src/deck/drivers/src/flowdeck_v1v2.c
 */
struct CameraData {
public:
	std::float_t delta_x, delta_y;
	std::float_t z;
	std::float_t yaw;
};

#endif /* CAMERADATA_HPP */
