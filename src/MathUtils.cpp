#include "MathUtils.hpp"
#include "Vec4.hpp"

float MathUtils::computeDirectionToBase(const Vec4 &pos, const Vec4 &init_pos) {
	const auto x = pos.x() + init_pos.x();
	const auto y = pos.y() + init_pos.y();
	float angle;

	if ((x < 0 && y < 0) || (x > 0 && y > 0)) {
		angle = (x < 0 ? PI_f / 2 : 3 * PI_f / 2) + std::abs(std::atan(y / x));
	} else {
		angle = (x < 0 ? 0.0F : PI_f) + std::abs(std::atan(x / y));
	}
	wrapToPi(&angle);
	return std::isnan(angle) ? 0.0F : angle;
}

void MathUtils::wrapToPi(float *val) {
	if (*val > PI_f) {
		*val -= 2 * PI_f;
	} else if (*val < -PI_f) {
		*val += 2 * PI_f;
	}
}
