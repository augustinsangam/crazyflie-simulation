#ifndef CAMERADATA_HPP
#define CAMERADATA_HPP

#include <cmath>

/*
 * https://github.com/bitcraze/crazyflie-firmware/blob/master/src/deck/drivers/src/zranger.c
 * https://github.com/bitcraze/crazyflie-firmware/blob/master/src/deck/drivers/src/flowdeck_v1v2.c
 */

struct CameraData {
public:
	std::float_t delta_x, delta_y;
	// TODO: std::uint16_t in mm
	std::float_t z;
};

#endif /* CAMERADATA_HPP */
