#ifndef CAMERADATA_HPP
#define CAMERADATA_HPP

#include <bits/stdint-uintn.h>

class CameraData {
public:
	uint16_t delta_x_l;
	uint16_t delta_x_h;
	uint16_t delta_y_l;
	uint16_t delta_y_h;
};
#endif
