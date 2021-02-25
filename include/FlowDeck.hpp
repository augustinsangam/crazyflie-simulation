#ifndef FLOWDECK_HPP
#define FLOWDECK_HPP

#include "CameraData.hpp"
#include "Vec4.hpp"
#include <cmath>

#include <argos3/core/utility/math/vector3.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>

class FlowDeck {
private:
	argos::CVector3 initial_pos_{};

public:
	inline void init(const argos::CVector3 &init_position) {
		initial_pos_ = init_position;
	}

	CameraData getInitPositionDelta(const argos::CVector3 &c_pos) {
		return CameraData{
		    static_cast<std::float_t>(c_pos.GetX() - initial_pos_.GetX()),
		    static_cast<std::float_t>(c_pos.GetY() - initial_pos_.GetY()),
		    static_cast<std::float_t>(c_pos.GetZ())};
	}
};

#endif /* FLOWDECK_HPP */
