#include "Brain.hpp"
#include "Constants.hpp"
#include "MathUtils.hpp"
#include "Vec4.hpp"
#include <spdlog/spdlog.h>

namespace brain {

/**
 * @brief x,y,z and yaw are the current position and orientation of the drone at
 * the time the function is called
 *
 * @param cd
 * @param sd
 * @param battery_level
 * @return std::optional<NextMove>
 */
std::optional<NextMove> Brain::computeNextMove(const CameraData *cd,
                                               const SensorData *sd,
                                               const double &battery_level) {
	cd_ = cd;
	sd_ = sd;
	float_t x = cd->delta_x + initial_pos_.x();
	float_t y = cd->delta_y + initial_pos_.y();
	float_t z = cd->z + initial_pos_.z();
	float_t yaw = cd->yaw;

	// spdlog::debug("f: {}, l: {}, b: {}, r: {}", sd->front, sd->left,
	// sd->back,
	//               sd->right);
	// spdlog::info("Desired Pos -> (x: " +
	// std::to_string(desired_position_.x())
	// +
	//              " y: " + std::to_string(desired_position_.y()) +
	//              " z: " + std::to_string(desired_position_.z()) +
	//              ") [tolerance=" + std::to_string(0.005) +
	//              "] yaw: " + std::to_string(desired_angle_));
	spdlog::info(
	    "[simulation_{}] Actual Pos -> (x: {}, y: {}, z: {}, yaw: {}), s={}",
	    id_, x, y, z, yaw, state_);

	NextMove nm{Vec4(0), true, desired_angle_};
	bool overwrite = false;

	doBatteryChecks(battery_level);

	switch (state_) {
	case State::idle: {
		nm = {Vec4(0), true, desired_angle_};
		break;
	}

	case State::take_off: {
		if (cd->z >= ALTITUDE_STEP * id_) {
			setupStabilization(Vec4(x, y, ALTITUDE_STEP * id_), yaw,
			                   State::auto_pilot);
			state_ = stabilize;
		}
		nm = {Vec4(x, y, ALTITUDE_STEP * id_), false, yaw};
		break;
	}

	case State::stabilize: {
		// spdlog::info("Stabilize");
		if (isStabilized(Vec4(yaw, x, y, z))) {
			state_ = next_state_;
			break;
		}
		nm = {desired_position_, false, desired_angle_};
		break;
	}

	case State::dodge: {
		// spdlog::info("dodge");
		if (!dodging_) {
			// "Starting rotation"
			float_t delta_angle = getDodgeRotation(sd, yaw);
			nm = {Vec4(x, y, z), false, yaw + delta_angle};
			dodging_ = true;
			break;
		}
		if (desired_angle_ - DODGE_PRECISION < yaw &&
		    yaw < desired_angle_ + DODGE_PRECISION) {
			// "Reached target angle. Stopping rotation"
			overwrite = true;
			nm = {Vec4(x, y, z), false, yaw};
			if (sd->front > WALL_DISTANCE_THRESH_FRONT) {
				// "Nothing in front of me, auto_pilot now"
				state_ = auto_pilot;
				dodge_type_ = unset;
			}
			// "Something still in front of me, restarting a rotation"
			dodging_ = false;
		}
		break;
	}

	case State::land: {
		if (cd->z < 0.01F) {
			state_ = State::idle;
			is_returning_to_base_ = false;
		}
		nm = {is_returning_to_base_ ? auto_pilot_target_position_
		                            : Vec4(x, y, 0),
		      false, desired_angle_};
		break;
	}

	case State::return_to_base: {
		spdlog::info("[simulation_{}] returning to base", id_);
		float angle =
		    MathUtils::computeDirectionToBase(Vec4(x, y, z), initial_pos_);
		auto_pilot_target_position_ =
		    Vec4(angle, initial_pos_.x(), initial_pos_.y(), 0);
		is_returning_to_base_ = true;
		nm = {Vec4(x, y, z), false, angle};
		setupStabilization(Vec4(x, y, z), angle, State::auto_pilot);
		state_ = stabilize;
		break;
	}

	case State::avoid_obstacle: {
		if (!avoiding_) {
			if (!dodging_) {
				float_t delta_angle = getDodgeRotation(sd, yaw);
				nm = {Vec4(x, y, z), false, yaw + delta_angle};
				dodging_ = true;
			} else {
				if (desired_angle_ - DODGE_PRECISION < yaw &&
				    yaw < desired_angle_ + DODGE_PRECISION) {
					overwrite = true;
					nm = {Vec4(x, y, z), false, yaw};
					if (sd->front > WALL_DISTANCE_THRESH_FRONT) {
						avoiding_ = true;
					}
					dodging_ = false;
				}
			}
			break;
		}
		if (++counter_ > 25) {
			counter_ = 0;
			nm = {
			    Vec4(x, y, z), false,
			    MathUtils::computeDirectionToBase(Vec4(x, y, z), initial_pos_)};
			avoiding_ = false;
			state_ = State::auto_pilot;
			break;
		}
		nm = {Vec4(0, -0.03F, 0), true, desired_angle_};
		break;
	}

	case State::auto_pilot: {
		if (sd->front < WALL_DISTANCE_THRESH_FRONT ||
		    sd->left < WALL_DISTANCE_THRESH_SIDE ||
		    sd->right < WALL_DISTANCE_THRESH_SIDE) {
			setupStabilization(Vec4(x, y, z), yaw,
			                   is_returning_to_base_ ? State::avoid_obstacle
			                                         : State::dodge);
			state_ = stabilize;
			break;
		}
		nm = {Vec4(0, -0.03F, 0), true, desired_angle_};
		overwrite = true;
		if (is_returning_to_base_) {
			if (auto_pilot_target_position_.x() - LAND_PRECISION < x &&
			    x < auto_pilot_target_position_.x() + LAND_PRECISION &&
			    auto_pilot_target_position_.y() - LAND_PRECISION < y &&
			    y < auto_pilot_target_position_.y() + LAND_PRECISION) {
				spdlog::info("[simulation_{}] amorcing landing now", id_);
				state_ = State::land;
			}
		}
		break;
	}
	}
	if (nm.coords == last_move_.coords && nm.relative == last_move_.relative &&
	    nm.yaw == last_move_.yaw && !overwrite) {
		return std::nullopt;
	}
	last_move_ = nm;
	return nm;
}

void Brain::setupStabilization(Vec4 position, float_t orientation,
                               State next_state) {
	desired_position_ = position;
	desired_angle_ = orientation;
	next_state_ = next_state;
}

float_t Brain::getDodgeRotation(const SensorData *sd,
                                const float_t &currentYaw) {
	float_t delta_angle;
	if (dodge_type_ == unset) {
		delta_angle = sd->right < sd->left ? MathUtils::PI_f / 12.0F
		                                   : -(MathUtils::PI_f / 12.0F);
		desired_angle_ = currentYaw + delta_angle;
		dodge_type_ = delta_angle <= 0 ? clockwise : counter_clockwise;
	} else {
		delta_angle = dodge_type_ == clockwise ? -MathUtils::PI_f / 12.0F
		                                       : MathUtils::PI_f / 12.0F;
		desired_angle_ = currentYaw + delta_angle;
	}
	MathUtils::wrapToPi(&desired_angle_);
	return delta_angle;
}

void Brain::doBatteryChecks(const double &battery_level) {
	if (battery_level < BATTERY_THRESHOLD_RTB && !is_returning_to_base_ &&
	    state_ != State::idle) {
		spdlog::info("[simulation_{}] low battery ({}%)", id_, battery_level);
		state_ = State::return_to_base;
	}

	if (battery_level < BATTERY_THRESHOLD_EMERGENCY && state_ != State::idle) {
		spdlog::warn("[simulation_{}] dead battery ({}%), amorcing "
		             "emergency landing",
		             id_, battery_level);
		state_ = land;
	}
}

bool Brain::isStabilized(const Vec4 &pos) {
	return desired_position_.x() - STABILIZE_POS_PRECISION < pos.x() &&
	       pos.x() < desired_position_.x() + STABILIZE_POS_PRECISION &&
	       desired_position_.y() - STABILIZE_POS_PRECISION < pos.y() &&
	       pos.y() < desired_position_.y() + STABILIZE_POS_PRECISION &&
	       desired_position_.z() - STABILIZE_POS_PRECISION < pos.z() &&
	       pos.z() < desired_position_.z() + STABILIZE_POS_PRECISION &&
	       std::abs(desired_angle_) - STABILIZE_YAW_PRECISION <
	           std::abs(pos.w()) &&
	       std::abs(pos.w()) <
	           std::abs(desired_angle_) + STABILIZE_YAW_PRECISION;
}

} // namespace brain
