#ifndef MATHUTILS_H
#define MATHUTILS_H

#include "SensorData.hpp"
#include "Vec4.hpp"
#include <cmath>

class MathUtils {
public:
	static constexpr float PI_f = 3.14159265F;

	static float computeDirectionToBase(const Vec4 &pos, const Vec4 &init_pos);

	static void wrapToPi(float *val);
};

#endif
