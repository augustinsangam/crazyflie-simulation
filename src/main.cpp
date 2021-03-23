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
#include "exploration/stabilizer_types.hpp"
#include "gen_buf.hpp"
#include <bits/stdint-uintn.h>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <ostream>
#include <random>
#include <string>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>

#include "exploration/state_machine.hpp"
#include "porting/porting.hpp"

#include <spdlog/spdlog.h>

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
static const double PI = 3.141582654;

/* Pointer to the position actuator */
static argos::CCI_QuadRotorPositionActuator *m_pcPropellers{};

/* Pointer to the battery sensor */
static argos::CCI_BatterySensor *m_pcBattery{};

/* Pointer to the positioning sensor */
static argos::CCI_PositioningSensor *m_pcPos{};

/* Pointer to the crazyflie distance sensor */
static argos::CCI_CrazyflieDistanceScannerSensor *m_pcDistance{};

static FlowDeck flow_deck_{};

class CCrazyflieSensing : public argos::CCI_Controller { // NOLINT
private:
	// 8 ticks per second
	static constexpr const uint8_t tick_rate_{8};
	// 2 pulses per second
	static constexpr const uint8_t pulse_rate_{2};
	// 1 pule each n ticks
	static constexpr const uint_fast8_t tick_pulse_{tick_rate_ / pulse_rate_};

	const uint16_t id_{mainId++};

	uint32_t tick_count_{};

	brain::Brain brain_;
	Decoder decoder_;
	RTStatus rt_status_;
	Proxy proxy_;
	exploration::StateMachine sm_;
	double orientation_ = argos::CRadians::PI_OVER_TWO.GetValue();

public:
	/* Class constructor. */
	CCrazyflieSensing()
	    : brain_(id_), decoder_(),
	      rt_status_("simulation_" + std::to_string(mainId)),
	      proxy_("simulation_" + std::to_string(mainId)) {
		brain_.setState(brain::idle);
		std::cout << "drone " << rt_status_.get_name() << " created"
		          << std::endl;
		sm_ = exploration::StateMachine();
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

		// #################################################################
		sm_.startup();
		// TODO merge single-connection in next, merge next in porting-v1
		// #################################################################
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
		// #################################################################
		/* Is that it ? */
		sm_.iteration_loop();

		// #################################################################
		// Battery
		const auto &battery = m_pcBattery->GetReading().AvailableCharge;

		// Position
		const auto &position = m_pcPos->GetReading().Position;
		const auto &orientation = m_pcPos->GetReading().Orientation;

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

	static uint8_t get_ticks_per_sec() { return tick_rate_; }
};

// NOLINTNEXTLINE
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")

// ####################### porting.hpp methods ###########################

uint64_t porting::us_timestamp() {
	// NOLINTNEXTLINE
	uint64_t us =
	    std::chrono::duration_cast<std::chrono::microseconds>(
	        std::chrono::high_resolution_clock::now().time_since_epoch())
	        .count();
	return us;
}

uint64_t porting::config_block_get_radio_address() {
	// works only for 1 drone
	return mainId;
}

void porting::system_wait_start() {}

void porting::ticks_delay(uint32_t nTicksToDelay) { usleep(nTicksToDelay); }

uint32_t porting::ms_to_ticks(uint16_t ms) {
	return ms * CCrazyflieSensing::get_ticks_per_sec() / 1000;
}

void porting::commander_set_setpoint(exploration::setpoint_t *setpoint,
                                     int priority) {
	/**
	 * [Notes]
	 * setpoint->mode.(x,y,z ou yaw) est soit modeAbs soit modeVelocity, ce qui
	 correspond en gros à setAbsolute et setRelative. Il y a enfin un
	 modeDisable pour tout couper

	 z est en modeVelocity pendant le décollage et l'atterrissage, et modeAbs
	 pendant la plupart du temps ("hover" "vel_command")


	 *
	 */
}

void porting::estimator_kalman_get_estimated_pos(exploration::point_t *pos) {
	const auto &position = m_pcPos->GetReading().Position;
	const auto &orientation = m_pcPos->GetReading().Orientation;
	CameraData cd = flow_deck_.getInitPositionDelta(position, orientation);
	pos->x = cd.delta_x;
	pos->y = cd.delta_y;
	pos->z = cd.z;
	pos->timestamp = static_cast<uint32_t>(std::time(nullptr));
}

bool porting::send_p2p_packet_broadcast(exploration::P2PPacket *p2pp) {
	// TODO
}

uint8_t porting::get_deck_bc_multiranger() {
	return static_cast<uint8_t>(m_pcDistance != nullptr);
}

uint8_t porting::get_deck_bc_flow2() {
	// TODO: check for real if the flowdeck is initialized properly
	return 1U;
}

float porting::get_kalman_state_z() {
	const auto &position = m_pcPos->GetReading().Position;
	const auto &orientation = m_pcPos->GetReading().Orientation;
	CameraData cd = flow_deck_.getInitPositionDelta(position, orientation);
	return cd.z;
}

float porting::get_stabilizer_yaw() {
	const auto &position = m_pcPos->GetReading().Position;
	const auto &orientation = m_pcPos->GetReading().Orientation;
	CameraData cd = flow_deck_.getInitPositionDelta(position, orientation);
	float_t yaw_rad = cd.yaw;
	return static_cast<float_t>(yaw_rad * static_cast<double>(180) / PI);
}

uint8_t porting::get_radio_rssi() {
	// TODO
}

float get_distance_sensor(uint8_t sensor_index) {
	/*
	 * From ci_crazyflie_distance_scanner_sensor.h
	 *      front
	 *
	 *        0
	 *
	 * left 1   3 right
	 *
	 *        2
	 *
	 *      back
	 */
	auto sDistRead = m_pcDistance->GetReadingsMap();
	auto iterDistRead = sDistRead.begin();
	if (sDistRead.size() == 4) {
		for (uint8_t i = 0; i < sensor_index; i++) {
			iterDistRead++;
		}
		return static_cast<float>((iterDistRead++)->second);
	}
	float default_value = 0.0;
	spdlog::error("Unable to read from the multiranger sensors! Returning a "
	              "default value (" +
	              std::to_string(default_value) + ").");
	return default_value;
}

float porting::get_front_range() { return get_distance_sensor(0); }
float porting::get_left_range() { return get_distance_sensor(1); }
float porting::get_back_range() { return get_distance_sensor(2); }
float porting::get_right_range() { return get_distance_sensor(3); }
float porting::get_up_range() {
	// TODO
	spdlog::error("Up sensor not implemented yet");
	return 1.0;
}
