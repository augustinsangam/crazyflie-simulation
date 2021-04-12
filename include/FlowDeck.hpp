#ifndef FLOWDECK_HPP
#define FLOWDECK_HPP

#include "CameraData.hpp"
#include "Vec4.hpp"
#include <cmath>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

/**
 * @brief FlowDeck : class that mimicks the behavior of a real crazyflie
 * flowdeck. It saves the initial position and returns a CameraData struct,
 * containing the delta_x and delta_y relative to the initial (takeoff) position
 *
 */
class FlowDeck {
private:
	argos::CVector3 initial_pos_{};

public:
	/**
	 * @brief Saves the takeoff position in case the drone does not takeoff at
	 * (0,0) in ARGoS. Called only when the simulation is launching.
	 *
	 * @param init_position real absolute position gotten from ARGoS directly
	 */
	inline void init(const argos::CVector3 &init_position) {
		initial_pos_ = init_position;
	}
	/**
	 * @brief Get the CameraData containing the relative movement done by the
	 * drone
	 *
	 * @param position real absolute position gotten from ARGoS directly
	 * @param orientation real absolute orientation gotten from ARGoS directly
	 * @return CameraData
	 */
	CameraData getInitPositionDelta(const argos::CVector3 &position,
	                                const argos::CQuaternion &orientation) {
		argos::CRadians z_angle, y_angle, x_angle;
		orientation.ToEulerAngles(z_angle, y_angle, x_angle);
		return CameraData{
		    static_cast<std::float_t>(position.GetX() - initial_pos_.GetX()),
		    static_cast<std::float_t>(position.GetY() - initial_pos_.GetY()),
		    static_cast<std::float_t>(position.GetZ()),
		    static_cast<float_t>(z_angle.GetValue())};
	}
};

#endif /* FLOWDECK_HPP */
