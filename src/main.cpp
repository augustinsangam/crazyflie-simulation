#include "Brain.hpp"
#include "CameraData.hpp"
#include "Decoder.hpp"
#include "FlowDeck.hpp"
#include "Proxy.hpp"
#include "RTStatus.hpp"
#include "SensorData.hpp"
#include "SharedQueue.hpp"
#include "Vec4.hpp"
#include "conn/Conn.hpp"
#include "gen_buf.hpp"
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

class CCrazyflieSensing : public argos::CCI_Controller { // NOLINT
private:
	// 8 ticks per second
	static constexpr const uint8_t tick_rate_{8};
	// 2 pulses per second
	static constexpr const uint8_t pulse_rate_{2};
	// 1 pule each n ticks
	static constexpr const uint_fast8_t tick_pulse_{tick_rate_ / pulse_rate_};

	const uint16_t id_{mainId++};

	exploration::StateMachine sm_;

	uint32_t tick_count_{};

	brain::Brain brain_;
	Decoder decoder_;
	RTStatus rt_status_;
	Proxy proxy_;
	double orientation_ = argos::CRadians::PI_OVER_TWO.GetValue();

	/* Pointer to the position actuator */
	argos::CCI_QuadRotorPositionActuator *m_pcPropellers{};

	/* Pointer to the battery sensor */
	argos::CCI_BatterySensor *m_pcBattery{};

	/* Pointer to the positioning sensor */
	argos::CCI_PositioningSensor *m_pcPos{};

	/* Pointer to the crazyflie distance sensor */
	argos::CCI_CrazyflieDistanceScannerSensor *m_pcDistance{};

	FlowDeck flow_deck_{};

public:
	/* Class constructor. */
	CCrazyflieSensing()
	    : brain_(id_), decoder_(),
	      rt_status_("simulation_" + std::to_string(mainId)),
	      proxy_("simulation_" + std::to_string(mainId)) {
		brain_.setState(brain::idle);
		std::cout << "drone " << rt_status_.get_name() << " created"
		          << std::endl;
	}

	/* Class destructor. */
	~CCrazyflieSensing() override = default;

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_diffusion_controller> section.
	 */
	void Init(argos::TConfigurationNode & /*t_node*/) override {
		spdlog::set_level(spdlog::level::trace);

		// TODO(): check
		sm_.init();

		/**/

		m_pcPropellers = GetActuator<argos::CCI_QuadRotorPositionActuator>(
		    "quadrotor_position");

		m_pcPos = GetSensor<argos::CCI_PositioningSensor>("positioning");

		m_pcBattery = GetSensor<argos::CCI_BatterySensor>("battery");
		m_pcDistance = GetSensor<argos::CCI_CrazyflieDistanceScannerSensor>(
		    "crazyflie_distance_scanner");

		const auto &cPos = m_pcPos->GetReading().Position;

		flow_deck_.init(cPos);
		brain_.setInitialPosition(Vec4(static_cast<float_t>(cPos.GetX()),
		                               static_cast<float_t>(cPos.GetY()),
		                               static_cast<float_t>(cPos.GetZ())));

		spdlog::info("argos_drone_" + std::to_string(id_) +
		             " init position: (x: " + std::to_string(cPos.GetX()) +
		             " y: " + std::to_string(cPos.GetY()) +
		             " z: " + std::to_string(cPos.GetZ()) + ")");
		spdlog::info("Init OK");
	}

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	void ControlStep() override {
		// TOOD(): check
		sm_.step();

		// Battery
		const auto &battery = m_pcBattery->GetReading().AvailableCharge;

		// Position
		const auto &position = m_pcPos->GetReading().Position;
		const auto &orientation = m_pcPos->GetReading().Orientation;

		// FlowDeck v2
		// https://www.bitcraze.io/products/flow-deck-v2/
		const auto camera_data =
		    flow_deck_.getInitPositionDelta(position, orientation);

		// Look here for documentation on the distance sensor:
		// /root/argos3/src/plugins/robots/crazyflie/control_interface/ci_crazyflie_distance_scanner_sensor.h
		// Read distance sensor
		// Multi-ranger deck
		// https://www.bitcraze.io/products/multi-ranger-deck/
		auto sDistRead = m_pcDistance->GetReadingsMap();
		auto iterDistRead = sDistRead.begin();
		SensorData sensor_data{};
		if (sDistRead.size() == 4) {
			sensor_data = {
			    static_cast<std::uint16_t>((iterDistRead++)->second),
			    static_cast<std::uint16_t>((iterDistRead++)->second),
			    static_cast<std::uint16_t>((iterDistRead++)->second),
			    static_cast<std::uint16_t>((iterDistRead++)->second)};
		}

		// spdlog::info(std::to_string(id_) +
		//              " -> (x: " + std::to_string(position.GetX()) +
		//              " y: " + std::to_string(position.GetY()) +
		//              " z: " + std::to_string(position.GetZ()) + ")");
		// Update drone status
		Vec4 position_vec4 = Vec4(static_cast<std::float_t>(position.GetX()),
		                          static_cast<std::float_t>(position.GetY()),
		                          static_cast<std::float_t>(position.GetZ()));
		rt_status_.update(static_cast<std::float_t>(battery), position_vec4);

		if (tick_count_ % tick_pulse_ == 0) {
			proxy_.send(rt_status_.encode());
		} else {
			auto cmd = proxy_.next_cmd();
			if (cmd) {
				spdlog::info("Received command {}", *cmd);
				switch (*cmd) {
				case cmd::take_off:
					brain_.setState(brain::State::take_off);
					rt_status_.enable();
					break;
				case cmd::land:
					brain_.setState(brain::State::land);
					rt_status_.disable();
					break;
				default:
					break;
				}
			}
		}
		++tick_count_;

		const auto next_move =
		    brain_.computeNextMove(&camera_data, &sensor_data);
		if (next_move) {
			// spdlog::info("next_move (" +
			// std::to_string(next_move->coords.x()) +
			//              "," + std::to_string(next_move->coords.y()) + "," +
			//              std::to_string(next_move->coords.z()) + ")" + " yaw
			//              " + std::to_string(next_move->yaw));
			if (next_move->relative) {
				m_pcPropellers->SetRelativePosition(argos::CVector3(
				    next_move->coords.x(), next_move->coords.y(),
				    next_move->coords.z()));
			} else {
				m_pcPropellers->SetAbsolutePosition(argos::CVector3(
				    next_move->coords.x(), next_move->coords.y(),
				    next_move->coords.z()));
				m_pcPropellers->SetAbsoluteYaw(argos::CRadians(next_move->yaw));
			}
		}
	}

	std::string trunc(float val, int numDigits) {
		std::string output = std::to_string(val).substr(0, numDigits + 1);
		if (output.find('.') == std::string::npos || output.back() == '.') {
			output.pop_back();
		}
		return output;
	}

	/*
	 * This function resets the controller to its state right after the
	 * Init().
	 * It is called when you press the reset button in the GUI.
	 * In this example controller there is no need for resetting anything,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	void Reset() override { tick_count_ = 0; }

	/*
	 * Called to cleanup what done by Init() when the experiment finishes.
	 * In this example controller there is no need for clean anything up,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	void Destroy() override { /*conn_->terminate();*/
	}
};

// NOLINTNEXTLINE
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")
