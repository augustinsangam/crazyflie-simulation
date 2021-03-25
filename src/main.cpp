#include "CCrazyflieSensing.hpp"

#include "Brain.hpp"
#include "CameraData.hpp"
#include "FlowDeck.hpp"
#include "Proxy.hpp"
#include "SensorData.hpp"
#include "SharedQueue.hpp"
#include "Vec4.hpp"
#include "conn/Conn.hpp"
#include "gen_buf.hpp"
#include "porting.hpp"
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <ostream>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>

#include <spdlog/spdlog.h>

#include <exploration/StateMachine.hpp>

#include <argos3/core/utility/math/angles.h>
#include <argos3/core/utility/math/quaternion.h>
/* argos::CCI_Controller */
#include <argos3/core/control_interface/ci_controller.h>
/* argos::TConfigurationNode */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* argos::Abs */
#include <argos3/core/utility/math/general.h>
/* argos::CVector3 */
#include <argos3/core/utility/math/vector3.h>
/* argos::CCI_BatterySensor */
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
/* argos::CCI_PositioningSensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* argos::CCI_QuadRotorPositionActuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
/* args::CCI_CrazyflieDistanceScannerSensor */
#include <argos3/plugins/robots/crazyflie/control_interface/ci_crazyflie_distance_scanner_sensor.h>

static uint16_t mainId = 0; // NOLINT

CCrazyflieSensing::CCrazyflieSensing()
    : id_(mainId++), porting_(this), sm_(&porting_), decoder_(),
      rt_status_("simulation_" + std::to_string(mainId)),
      proxy_("simulation_" + std::to_string(mainId)) {
	std::cout << "drone " << rt_status_.get_name() << " created" << std::endl;
}

void CCrazyflieSensing::Init(argos::TConfigurationNode & /*t_node*/) {
	spdlog::set_level(spdlog::level::trace);

	// TODO(): Launch a thread and call sm_.p2p_call_back when needed

	sm_.init(); // TODO(): check

	m_pcPropellers =
	    GetActuator<argos::CCI_QuadRotorPositionActuator>("quadrotor_position");

	m_pcPos = GetSensor<argos::CCI_PositioningSensor>("positioning");

	m_pcBattery = GetSensor<argos::CCI_BatterySensor>("battery");
	m_pcDistance = GetSensor<argos::CCI_CrazyflieDistanceScannerSensor>(
	    "crazyflie_distance_scanner");

	const auto &cPos = m_pcPos->GetReading().Position;

	flow_deck_.init(cPos);

	spdlog::info("argos_drone_" + std::to_string(id_) +
	             " init position: (x: " + std::to_string(cPos.GetX()) +
	             " y: " + std::to_string(cPos.GetY()) +
	             " z: " + std::to_string(cPos.GetZ()) + ")");
	spdlog::info("Init OK");
}

void CCrazyflieSensing::ControlStep() {
	if (tick_count_ % tick_pulse_ == 0) {
		proxy_.send(rt_status_.encode());
		sm_.step(); // TODO(): Check.
	} else {
		auto cmd = proxy_.next_cmd();
		if (cmd) {
			spdlog::info("Received command {}", *cmd);
		}
	}
	++tick_count_;
}

void CCrazyflieSensing::Reset() { tick_count_ = 0; }

void CCrazyflieSensing::Destroy() { /*conn_->terminate();*/
}

// NOLINTNEXTLINE
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")
