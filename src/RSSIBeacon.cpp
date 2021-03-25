#include "RSSIBeacon.hpp"

void RSSIBeacon::init(Vec4 initPos) { init_position_ = initPos; }

/**
 * @brief Computes the distance between the start beacon positionned
 at init_position_ and the drone current position :
    sqrt((x_i - x_d)^2 + (y_i - y_d)^2 + (z_i - z_d)^2)
 *
 * @param dronePos actual drone position
 */
void RSSIBeacon::update(Vec4 dronePos) {
	beam_ = sqrt(pow(init_position_.x(), dronePos.x()) +
	             pow(init_position_.y(), dronePos.y()) +
	             pow(init_position_.z(), dronePos.z()));
}

float_t RSSIBeacon::read_value() const { return beam_; }
