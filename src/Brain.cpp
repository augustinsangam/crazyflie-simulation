#include "Brain.hpp"
#include "Constants.hpp"

namespace brain {

std::optional<NextMove> Brain::computeNextMove(const CameraData *cd,
                                               const SensorData *sd,
                                               const double &battery_level) {
	cd_ = cd;
	sd_ = sd;
	float_t x = cd->delta_x + initial_pos_.x();
	float_t y = cd->delta_y + initial_pos_.y();
	float_t z = cd->z + initial_pos_.z();

	// spdlog::debug("f: {}, l: {}, b: {}, r: {}", sd->front, sd->left,
	// sd->back,
	//               sd->right);
	// spdlog::info("Desired Pos -> (x: " + std::to_string(desiredPosition_.x())
	// +
	//              " y: " + std::to_string(desiredPosition_.y()) +
	//              " z: " + std::to_string(desiredPosition_.z()) +
	//              ") [tolerance=" + std::to_string(0.005) +
	//              "] yaw: " + std::to_string(desiredAngle_));
	spdlog::info("[simulation_{}] Actual Pos -> (x: {}, y: {}, z: {}, yaw: {}",
	             id_, x, y, z, cd->yaw);

	NextMove nm{Vec4(0), true, desiredAngle_};
	bool overwrite = false;

	if (battery_level < battery_threshold_ && !is_returning_to_base_ &&
	    state_ != State::idle) {
		spdlog::info("[simulation_{}] low battery ({}%)", id_, battery_level);
		setState(State::return_to_base);
	}

	switch (state_) {
	case State::idle:
		// spdlog::info("Idle");
		nm = {Vec4(0), true, desiredAngle_};
		break;

	case State::take_off:
		// spdlog::info("Take Off");
		if (cd->z >= ALTITUDE_STEP * id_) {
			setupStabilization(Vec4(x, y, ALTITUDE_STEP * id_), cd->yaw,
			                   State::auto_pilot);
			state_ = stabilize;
		}
		nm = {Vec4(x, y, ALTITUDE_STEP * id_), false, cd->yaw};
		break;

	case State::stabilize:
		// spdlog::info("Stabilize");
		if (desiredPosition_.x() - STABILIZE_POS_PRECISION < x &&
		    x < desiredPosition_.x() + STABILIZE_POS_PRECISION &&
		    desiredPosition_.y() - STABILIZE_POS_PRECISION < y &&
		    y < desiredPosition_.y() + STABILIZE_POS_PRECISION &&
		    desiredPosition_.z() - STABILIZE_POS_PRECISION < z &&
		    z < desiredPosition_.z() + STABILIZE_POS_PRECISION &&
		    std::abs(desiredAngle_) - STABILIZE_YAW_PRECISION <
		        std::abs(cd->yaw) &&
		    std::abs(cd->yaw) <
		        std::abs(desiredAngle_) + STABILIZE_YAW_PRECISION) {
			state_ = afterStab_;
		} else {
			nm = {desiredPosition_, false, desiredAngle_};
		}
		break;

	case State::dodge:
		// spdlog::info("dodge");
		if (!dodging_) {
			// "Starting rotation"
			float_t delta_angle = getDodgeRotation(sd, cd->yaw);
			nm = {Vec4(x, y, z), false, cd->yaw + delta_angle};
			dodging_ = true;
		} else {
			if (desiredAngle_ - DODGE_PRECISION < cd->yaw &&
			    cd->yaw < desiredAngle_ + DODGE_PRECISION) {
				// "Reached target angle. Stopping rotation"
				overwrite = true;
				nm = {Vec4(x, y, z), false, cd->yaw};
				if (sd->front > WALL_DISTANCE_THRESH_FRONT) {
					// "Nothing in front of me, auto_pilot now"
					state_ = auto_pilot;
					dodge_type_ = unset;
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
			is_returning_to_base_ = false;
		}
		nm = {is_returning_to_base_ ? auto_pilot_target_position_
		                            : Vec4(x, y, 0),
		      false, desiredAngle_};
		break;

	case State::return_to_base: {
		spdlog::info("[simulation_{}] returning to base", id_);
		float angle = computeDirectionToBase(Vec4(x, y, z));
		auto_pilot_target_position_ =
		    Vec4(angle, initial_pos_.x(), initial_pos_.y(), 0);
		is_returning_to_base_ = true;
		nm = {Vec4(x, y, z), false, angle};
		setupStabilization(Vec4(x, y, z), angle, State::auto_pilot);
		state_ = stabilize;
		break;
	}

	case State::avoid_obstacle: {
		// spdlog::info("avoid_obstacle");
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
				if (desiredAngle_ - DODGE_PRECISION < cd->yaw &&
				    cd->yaw < desiredAngle_ + DODGE_PRECISION) {
					// "Reached target angle. Stopping rotation"
					overwrite = true;
					nm = {Vec4(x, y, z), false, cd->yaw};
					if (sd->front > WALL_DISTANCE_THRESH_FRONT) {
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
			// advance for a while
			nm = {Vec4(0, -0.03F, 0), true, desiredAngle_};
		}
		break;
	}

	case State::auto_pilot: {
		// spdlog::info("auto_pilot");
		if (sd->front < WALL_DISTANCE_THRESH_FRONT ||
		    sd->left < WALL_DISTANCE_THRESH_SIDE ||
		    sd->right < WALL_DISTANCE_THRESH_SIDE) {
			setupStabilization(Vec4(x, y, z), cd->yaw,
			                   is_returning_to_base_ ? State::avoid_obstacle
			                                         : State::dodge);
			state_ = stabilize;
			break;
		}
		nm = {Vec4(0, -0.03F, 0), true, desiredAngle_};
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

float Brain::computeDirectionToBase(const Vec4 &pos) const {
	const auto x = pos.x();
	const auto y = pos.y();
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
	return std::isnan(angle) ? 0.0F : angle;
}

float_t Brain::getDodgeRotation(const SensorData *sd,
                                const float_t &currentYaw) {
	float_t delta_angle;
	if (dodge_type_ == unset) {
		delta_angle = sd->right < sd->left ? PI / 12.0F : -(PI / 12.0F);
		desiredAngle_ = currentYaw + delta_angle;
		dodge_type_ = delta_angle <= 0 ? clockwise : counter_clockwise;
	} else {
		delta_angle = dodge_type_ == clockwise ? -PI / 12.0F : PI / 12.0F;
		desiredAngle_ = currentYaw + delta_angle;
	}
	if (desiredAngle_ > PI) {
		desiredAngle_ -= 2 * PI;
	} else if (desiredAngle_ < -PI) {
		desiredAngle_ += 2 * PI;
	}
	return delta_angle;
}

} // namespace brain
