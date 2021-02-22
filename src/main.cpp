#include "Brain.hpp"
#include "CameraData.hpp"
#include "Conn.hpp"
#include "Decoder.hpp"
#include "FlowDeck.hpp"
#include "RTStatus.hpp"
#include "SensorData.hpp"
#include <cstdint>
#include <exception>
#include <iostream>
#include <ostream>
#include <random>
#include <string>
#include <thread>
#include <utility>

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
/* Definition of the crazyflie distance sensor */
#include <argos3/plugins/robots/crazyflie/control_interface/ci_crazyflie_distance_scanner_sensor.h>

static uint16_t mainId = 0;

class CCrazyflieSensing : public argos::CCI_Controller {
private:
	// 8 ticks per second
	static constexpr const uint8_t tick_rate_{8};
	// 2 pulses per second
	static constexpr const uint8_t pulse_rate_{2};
	// 1 pule each n ticks
	static constexpr const uint_fast8_t tick_pulse_{tick_rate_ / pulse_rate_};

	uint32_t tick_count_{};
	conn::Conn conn_;
	RTStatus rt_status_;
	uint16_t id_ = mainId++;
	Decoder decoder_;
	int counter_ = 0;
	int squareSize_ = 5;
	brain::Brain brain_;
	std::array<argos::CVector3, 4> squareMoves_ = {
	    argos::CVector3(1, 0, 0.5), argos::CVector3(0, 1, 0.5),
	    argos::CVector3(-1, 0, 0.5), argos::CVector3(0, -1, 0.5)};

	/* Pointer to the position actuator */
	argos::CCI_QuadRotorPositionActuator *m_pcPropellers{};

	/* Pointer to the positioning sensor */
	argos::CCI_PositioningSensor *m_pcPos{};

	/* Pointer to the crazyflie distance sensor */
	argos::CCI_CrazyflieDistanceScannerSensor *m_pcDistance;

	/* Pointer to the battery sensor */
	argos::CCI_BatterySensor *m_pcBattery{};

	FlowDeck flow_deck_{};

public:
	/* Class constructor. */
	CCrazyflieSensing()
	    : conn_("localhost", 3995),
	      rt_status_("argos_drone_" + std::to_string(mainId)), decoder_(),
	      m_pcDistance(NULL), brain_() {
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
		m_pcPropellers = GetActuator<argos::CCI_QuadRotorPositionActuator>(
		    "quadrotor_position");

		m_pcPos = GetSensor<argos::CCI_PositioningSensor>("positioning");

		m_pcBattery = GetSensor<argos::CCI_BatterySensor>("battery");
		m_pcDistance = GetSensor<argos::CCI_CrazyflieDistanceScannerSensor>(
		    "crazyflie_distance_scanner");

		const auto &cPos = m_pcPos->GetReading().Position;
		flow_deck_.init(cPos);

		// TODO : Better logging
		std::cout << "Init OK" << std::endl;
	}

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	void ControlStep() override {

		// Battery
		const auto &sBatRead = m_pcBattery->GetReading();

		// Position
		const auto &c_pos = m_pcPos->GetReading().Position;

		// FlowDeck v2
		// https://www.bitcraze.io/products/flow-deck-v2/
		const auto camera_data = flow_deck_.getInitPositionDelta(c_pos);

		// Look here for documentation on the distance sensor:
		// /root/argos3/src/plugins/robots/crazyflie/control_interface/ci_crazyflie_distance_scanner_sensor.h
		// Read distance sensor
		// Multi-ranger deck
		// https://www.bitcraze.io/products/multi-ranger-deck/
		const auto &dist_data = m_pcDistance->GetLongReadingsMap();
		auto dist_data_it = dist_data.begin();
		SensorData sensor_data{
		    static_cast<std::uint16_t>((dist_data_it++)->second),
		    static_cast<std::uint16_t>((dist_data_it++)->second),
		    static_cast<std::uint16_t>((dist_data_it++)->second),
		    static_cast<std::uint16_t>((dist_data_it++)->second)};

		auto i = 0;
		for (auto it = dist_data.begin(); it != dist_data.end(); ++it, ++i)
			;

		// Update drone status
		rt_status_.update(static_cast<std::float_t>(sBatRead.AvailableCharge),
		                  Vec4(static_cast<std::float_t>(c_pos.GetX()),
		                       static_cast<std::float_t>(c_pos.GetY()),
		                       static_cast<std::float_t>(c_pos.GetZ())));

		switch (conn_.status()) {
		case conn::connected:
			if (tick_count_ % tick_pulse_ == 0) {
				conn_.send(rt_status_.encode());
			} else {
				auto msg = conn_.recv();
				if (msg) {
					auto cmd = decoder_.decode(std::move(*msg));
					if (cmd) {
						switch (*cmd) {
						case take_off:
							brain_.setState(brain::State::take_off);
							break;

						case land:
							brain_.setState(brain::State::land);
							break;

						default:
							break;
						}
					}
				}
			}
			break;
		case conn::plugable:
			conn_.plug();
			break;
		case conn::connectable:
			conn_.connect();
			break;
		case conn::unplugable:
			conn_.unplug();
			break;
		}

		Vec4 next_move = brain_.computeNextMove(&camera_data, &sensor_data);
		m_pcPropellers->SetRelativePosition(
		    argos::CVector3(next_move.x(), next_move.y(), next_move.z()));

		++tick_count_;
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
	void Destroy() override {}
};

// NOLINTNEXTLINE
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")
