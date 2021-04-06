#include "Brain.hpp"
#include "SensorData.hpp"
#include "Vec4.hpp"
#include <argos3/core/utility/math/angles.h>
#include <bits/stdint-uintn.h>
#include <cmath>
#include <cstdlib>
#include <optional>
#include <spdlog/spdlog.h>
#include <string>

namespace brain {

std::optional<NextMove> Brain::computeNextMove(const CameraData *cd,
                                               const SensorData *sd) {
	cd_ = cd;
	sd_ = sd;
	float_t x = cd->delta_x + initial_pos_.x();
	float_t y = cd->delta_y + initial_pos_.y();
	float_t z = cd->z + initial_pos_.z();

	uint16_t sensor_wall_distance_thresh_front = 15;
	uint16_t sensor_wall_distance_thresh_side =
	    sensor_wall_distance_thresh_front - 10;
	spdlog::debug("f: {}, l: {}, b: {}, r: {}", sd->front, sd->left, sd->back,
	              sd->right);
	// spdlog::info("Desired Pos -> (x: " + std::to_string(desiredPosition_.x())
	// +
	//              " y: " + std::to_string(desiredPosition_.y()) +
	//              " z: " + std::to_string(desiredPosition_.z()) +
	//              ") [tolerance=" + std::to_string(0.005) +
	//              "] yaw: " + std::to_string(desiredAngle_));
	spdlog::info("Actual Pos -> (x: {}, y: {}, z: {}, yaw: {}", x, y, z,
	             cd->yaw);

	NextMove nm{Vec4(0), true, desiredAngle_};
	bool overwrite = false;

	switch (state_) {
	case State::idle:
		// spdlog::info("Idle");
		nm = {Vec4(0), true, desiredAngle_};
		break;

	case State::take_off:
		// spdlog::info("Take Off");
		if (cd->z >= 0.5F) {
			setupStabilization(Vec4(x, y, z), cd->yaw, State::auto_pilot);
			state_ = stabilize;
		}
		nm = {Vec4(0, 0, 0.5F), true, desiredAngle_};
		break;

	case State::stabilize:
		spdlog::info("Stabilize");
		/* (debug)
		if (desiredPosition_.x() - 0.005F < cd->delta_x &&
		    cd->delta_x < desiredPosition_.x() + 0.005F) {
		    spdlog::info("x ok");
		}
		if (desiredPosition_.y() - 0.005F < cd->delta_y &&
		    cd->delta_y < desiredPosition_.y() + 0.005F) {
		    spdlog::info("y ok");
		}
		if (desiredPosition_.z() - 0.005F < z &&
		    z < desiredPosition_.z() + 0.005F) {
		    spdlog::info("z ok");
		}
		if (desiredAngle_ - 0.01F < cd->yaw &&
		    cd->yaw < desiredAngle_ + 0.01F) {
		    spdlog::info("yaw ok");
		}
		*/
		if (desiredPosition_.x() - 0.005F < cd->delta_x &&
		    cd->delta_x < desiredPosition_.x() + 0.005F &&
		    desiredPosition_.y() - 0.005F < cd->delta_y &&
		    cd->delta_y < desiredPosition_.y() + 0.005F &&
		    desiredPosition_.z() - 0.005F < z &&
		    z < desiredPosition_.z() + 0.005F &&
		    std::abs(desiredAngle_) - 0.01F < std::abs(cd->yaw) &&
		    std::abs(cd->yaw) < std::abs(desiredAngle_) + 0.01F) {
			state_ = afterStab_;
		} else {
			nm = {desiredPosition_, false, desiredAngle_};
		}
		break;

	case State::dodge:
		// spdlog::info("dodge");
		if (!dodging_) {
			// "Starting rotation"
			float_t delta_angle =
			    sd->right < sd->left ? PI / 12.0F : -(PI / 12.0F);
			desiredAngle_ = cd->yaw + delta_angle;
			if (desiredAngle_ > PI) {
				desiredAngle_ -= 2 * PI;
			} else if (desiredAngle_ < -PI) {
				desiredAngle_ += 2 * PI;
			}
			nm = {Vec4(x, y, z), false, cd->yaw + delta_angle};
			dodging_ = true;
		} else {
			// spdlog::info("Rotating (current " + std::to_string(cd->yaw) +
			//              " target " + std::to_string(desiredAngle_) + ")");
			if (desiredAngle_ - 0.05F < cd->yaw &&
			    cd->yaw < desiredAngle_ + 0.05F) {
				// "Reached target angle. Stopping rotation"
				overwrite = true;
				nm = {Vec4(x, y, z), false, cd->yaw};
				if (sd->front > sensor_wall_distance_thresh_front) {
					// "Nothing in front of me, auto_pilot now"
					state_ = auto_pilot;
				}
				// "Something still in front of me, restarting a rotation"
				dodging_ = false;
			}
		}
		break;

	case State::land:
		// spdlog::info("Land");
		if (cd->z < 0.01F) {
			state_ = State::idle;
			target_position_is_set_ = false;
		}
		nm = {target_position_is_set_ ? auto_pilot_target_position_
		                              : Vec4(0, 0, 0),
		      false, desiredAngle_};
		break;

	case State::return_to_base: {
		spdlog::debug("Return to base");
		float angle = computeDirectionToBase(Vec4(x, y, z));
		spdlog::debug("Angle to base = {}", angle * 180.0F / PI);
		auto_pilot_target_position_ =
		    Vec4(angle, initial_pos_.x(), initial_pos_.y(), 0);
		target_position_is_set_ = true;
		spdlog::debug("Target position = x: {}, y: {}, angle: {}",
		              auto_pilot_target_position_.x(),
		              auto_pilot_target_position_.y(),
		              auto_pilot_target_position_.w());
		nm = {Vec4(x, y, z), false, angle};
		setupStabilization(Vec4(x, y, z), angle, State::auto_pilot);
		state_ = stabilize;
		break;
	}

	case State::avoid_obstacle: {
		spdlog::info("avoid_obstacle");
		if (!avoiding_) {
			if (!dodging_) {
				// "Starting rotation"
				float_t delta_angle;
				if (sd->right > 1024 || sd->left > 1024) {
					delta_angle = PI / 12.0F;
				} else {
					delta_angle =
					    sd->right < sd->left ? PI / 12.0F : -(PI / 12.0F);
				}
				desiredAngle_ = cd->yaw + delta_angle;
				if (desiredAngle_ > PI) {
					desiredAngle_ -= 2 * PI;
				} else if (desiredAngle_ < -PI) {
					desiredAngle_ += 2 * PI;
				}
				nm = {Vec4(x, y, z), false, cd->yaw + delta_angle};
				dodging_ = true;
			} else {
				// spdlog::info("Rotating (current " + std::to_string(cd->yaw) +
				//              " target " + std::to_string(desiredAngle_) +
				//              ")");
				if (desiredAngle_ - 0.05F < cd->yaw &&
				    cd->yaw < desiredAngle_ + 0.05F) {
					// "Reached target angle. Stopping rotation"
					overwrite = true;
					nm = {Vec4(x, y, z), false, cd->yaw};
					if (sd->front > sensor_wall_distance_thresh_front) {
						// "Nothing in front of me, correct the orientation and
						// auto_pilot now"
						avoiding_ = true;
					}
					// "Something still in front of me, restarting a rotation"
					dodging_ = false;
				}
			}
		} else {
			// nothing in front of me, now start to advance for a while
			if (++counter_ > 25) {
				// stop and try again to go to the base
				counter_ = 0;
				nm = {Vec4(x, y, z), false,
				      computeDirectionToBase(Vec4(x, y, z))};
				avoiding_ = false;
				state_ = State::auto_pilot;
				break;
			}
			spdlog::debug("b careful");
			// advance for a while
			nm = {Vec4(0, -0.03F, 0), true, desiredAngle_};
		}
		break;
	}

	case State::auto_pilot: {
		// spdlog::info("auto_pilot");
		if (sd->front < sensor_wall_distance_thresh_front ||
		    sd->left < sensor_wall_distance_thresh_side ||
		    sd->right < sensor_wall_distance_thresh_side) {
			setupStabilization(Vec4(x, y, z), cd->yaw,
			                   target_position_is_set_ ? State::avoid_obstacle
			                                           : State::dodge);
			state_ = stabilize;
			break;
		}
		nm = {Vec4(0, -0.03F, 0), true, desiredAngle_};
		overwrite = true;
		float angle = computeDirectionToBase(Vec4(x, y, z)) * 180.0F / PI;
		spdlog::debug("Angle to base = {}", angle);
		if (target_position_is_set_) {
			spdlog::debug("Target position = x: {}, y: {}, angle: {}",
			              auto_pilot_target_position_.x(),
			              auto_pilot_target_position_.y(),
			              auto_pilot_target_position_.w());
			if (auto_pilot_target_position_.x() - 0.03F < cd->delta_x &&
			    cd->delta_x < auto_pilot_target_position_.x() + 0.03F &&
			    auto_pilot_target_position_.y() - 0.03F < cd->delta_y &&
			    cd->delta_y < auto_pilot_target_position_.y() + 0.03F) {
				// auto_pilot_target_position_.z() - 0.01F < z &&
				// z < auto_pilot_target_position_.z() + 0.01F) {
				spdlog::debug("==================================");
				state_ = State::land;
			}
		}
		break;
	}
	}
	if (nm.coords == lastMove_.coords && nm.relative == lastMove_.relative &&
	    nm.yaw == lastMove_.yaw && !overwrite) {
		return std::nullopt;
	}
	lastMove_ = nm;
	return nm;
}

void Brain::setupStabilization(Vec4 position, float_t orientation,
                               State next_state) {
	desiredPosition_ = position;
	desiredAngle_ = orientation;
	afterStab_ = next_state;
}

float Brain::computeDirectionToBase(const Vec4 &pos) {
	const auto x = pos.x();
	const auto y = pos.y();
	const auto z = pos.z();
	float angle;

	if ((x < 0 && y < 0) || (x > 0 && y > 0)) {
		angle = (x < 0 ? PI / 2 : 3 * PI / 2) + std::abs(std::atan(y / x));
	} else {
		angle = (x < 0 ? 0.0F : PI) + std::abs(std::atan(x / y));
	}
	if (angle > PI) {
		angle -= 2 * PI;
	} else if (angle < -PI) {
		angle += 2 * PI;
	}
	spdlog::debug("Computed angle = {}", angle);
	return angle;
}

void Brain::startReturnToBase() {
	spdlog::debug("Starting to return to base");
	if (state_ != stabilize) {
		setupStabilization(Vec4(cd_->delta_x + initial_pos_.x(),
		                        cd_->delta_y + initial_pos_.y(),
		                        cd_->z + initial_pos_.z()),
		                   cd_->yaw, State::return_to_base);
		state_ = stabilize;
	} else {
		afterStab_ = State::return_to_base;
	}
}

} // namespace brain
