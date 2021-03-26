#include "Brain.hpp"
#include "CameraData.hpp"
#include "Decoder.hpp"
#include "FlowDeck.hpp"
#include "Proxy.hpp"
#include "RSSIBeacon.hpp"
#include "RTStatus.hpp"
#include "SensorData.hpp"
#include "SharedQueue.hpp"
#include "Vec4.hpp"
#include "conn/Conn.hpp"
#include "exploration/types.hpp"
#include "gen_buf.hpp"
#include <argos3/core/utility/datatypes/datatypes.h>
#include <bits/stdint-uintn.h>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <memory>
#include <ostream>
#include <random>
#include <string>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <utility>

/* exploration */
#include <exploration/StateMachine.hpp>
#include <porting.hpp>

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

static uint16_t mainId = 1; // NOLINT
static const double PI = 3.141582654;

class CCrazyflieSensing : public argos::CCI_Controller { // NOLINT
private:
	enum crazyflie_sensing_operation {
		peer_to_peer_communication,
		tcp_socket_communication_send,
		tcp_socket_communication_recv,
		state_machine,
		idle
	};
	// 8 ticks per second
	static constexpr const uint8_t tick_rate_{16};

	uint32_t tick_count_{};

	porting::DroneLayer layer_{this};
	exploration::StateMachine sm_{&layer_};

	Decoder decoder_;
	RTStatus rt_status_;
	Proxy proxy_;
	double orientation_ = argos::CRadians::PI_OVER_TWO.GetValue();

	crazyflie_sensing_operation current_operation_{idle};
	static constexpr uint8_t idle_operation_ticks_ = 2;

public:
	const uint16_t id_{++mainId};

	/* Pointer to the position actuator */
	argos::CCI_QuadRotorPositionActuator *m_pcPropellers{};

	/* Pointer to the battery sensor */
	argos::CCI_BatterySensor *m_pcBattery{};

	/* Pointer to the positioning sensor */
	argos::CCI_PositioningSensor *m_pcPos{};

	/* Pointer to the crazyflie distance sensor */
	argos::CCI_CrazyflieDistanceScannerSensor *m_pcDistance{};

	FlowDeck flow_deck_{};

	RSSIBeacon rssi_beacon_{};

	/* Class constructor. */
	CCrazyflieSensing()
	    : decoder_(), rt_status_("simulation_" + std::to_string(mainId)),
	      proxy_("simulation_" + std::to_string(mainId)) {
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

		/* init brain */
		sm_.init();

		m_pcPropellers = GetActuator<argos::CCI_QuadRotorPositionActuator>(
		    "quadrotor_position");

		m_pcPos = GetSensor<argos::CCI_PositioningSensor>("positioning");

		m_pcBattery = GetSensor<argos::CCI_BatterySensor>("battery");
		m_pcDistance = GetSensor<argos::CCI_CrazyflieDistanceScannerSensor>(
		    "crazyflie_distance_scanner");

		const auto &cPos = m_pcPos->GetReading().Position;

		flow_deck_.init(cPos);
		rssi_beacon_.init(Vec4(static_cast<float_t>(cPos.GetX()),
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
		/* ##################### Update sensors ##################### */
		// Battery
		const auto &battery = m_pcBattery->GetReading().AvailableCharge;

		// Position
		const auto &position = m_pcPos->GetReading().Position;
		const auto &orientation = m_pcPos->GetReading().Orientation;

		// FlowDeck v2
		// https://www.bitcraze.io/products/flow-deck-v2/
		const auto camera_data =
		    flow_deck_.getInitPositionDelta(position, orientation);

		// Update rssi_beacon
		rssi_beacon_.update(Vec4(static_cast<float_t>(position.GetX()),
		                         static_cast<float_t>(position.GetY()),
		                         static_cast<float_t>(position.GetZ())));

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

		spdlog::debug("{} -> (x: {}, y: {}, z: {})", id_, position.GetX(),
		              position.GetY(), position.GetZ());
		// Update drone status
		Vec4 position_vec4 = Vec4(static_cast<std::float_t>(position.GetX()),
		                          static_cast<std::float_t>(position.GetY()),
		                          static_cast<std::float_t>(position.GetZ()));
		rt_status_.update(static_cast<std::float_t>(battery), position_vec4);

		/* ############## Update actuators and communication ############## */

		if (tick_count_ % 16 == 0 || tick_count_ % 16 == 8) {
			current_operation_ = tcp_socket_communication_send;
		} else if (tick_count_ % 16 == 3 || tick_count_ % 16 == 9 ||
		           tick_count_ % 16 == 15) {
			current_operation_ = state_machine;
		} else {
			if (current_operation_ != tcp_socket_communication_recv &&
			    current_operation_ != peer_to_peer_communication) {
				current_operation_ = tcp_socket_communication_recv;
			} else if (current_operation_ == tcp_socket_communication_recv) {
				current_operation_ = peer_to_peer_communication;
			} else {
				current_operation_ = tcp_socket_communication_recv;
			}
		}

		switch (current_operation_) {
		case peer_to_peer_communication: {
			// TODO
			break;
		}
		case tcp_socket_communication_send: {
			proxy_.send(rt_status_.encode());
			break;
		}
		case tcp_socket_communication_recv: {
			auto cmd = proxy_.next_cmd();
			if (!cmd) {
				break;
			}
			spdlog::info("Received command {}", *cmd);
			switch (*cmd) {
			case cmd::take_off: {
				spdlog::info("take_off received");
				// TODO delete pointer somewhere
				auto *p = new exploration::P2PPacket{};
				p->size = 0xff;
				p->rssi = 0x80;
				// p->port = 0x63;
				p->data[0] = 0x63;
				p->data[1] = 0x01;
				sm_.p2p_callback_handler(p);
				rt_status_.enable();
				break;
			}
			case cmd::land: {
				rt_status_.disable();
				break;
			}
			default:
				break;
			}
			break;
		}
		case state_machine: {
			sm_.step();
			break;
		}
		case idle: {
			break;
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
};

namespace porting {

std::uint64_t timestamp_us() {
	// NOLINTNEXTLINE
	uint64_t us =
	    std::chrono::duration_cast<std::chrono::microseconds>(
	        std::chrono::high_resolution_clock::now().time_since_epoch())
	        .count();
	return us;
}

void DroneLayer::kalman_estimated_pos(exploration::point_t *pos) {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	const auto &position = cf->m_pcPos->GetReading().Position;
	const auto &orientation = cf->m_pcPos->GetReading().Orientation;
	CameraData cd = cf->flow_deck_.getInitPositionDelta(position, orientation);
	pos->x = cd.delta_x;
	pos->y = cd.delta_y;
	pos->z = cd.z;
	pos->timestamp = static_cast<uint32_t>(std::time(nullptr));
}

void DroneLayer::radiolink_broadcast_packet(exploration::P2PPacket *packet) {
	// TODO()
}

void DroneLayer::DroneLayer::system_wait_start() {}

void DroneLayer::delay_ms(uint32_t t_ms) { usleep(t_ms * 1000); }

void DroneLayer::commander_set_point(exploration::setpoint_t *sp, int prio) {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);

	auto angle = sp->attitudeRate.yaw * PI / 180;
	/* yaws */
	if (sp->mode.yaw == exploration::modeVelocity) {
		spdlog::debug("yaw-relative: {}", angle);
		cf->m_pcPropellers->SetRelativeYaw(
		    argos::CRadians(static_cast<double>(angle)));
	} else if (sp->mode.yaw == exploration::modeAbs) {
		spdlog::debug("yaw-absolute {}", angle);
		cf->m_pcPropellers->SetAbsoluteYaw(
		    argos::CRadians(static_cast<double>(angle)));
	}

	/* take_of/land */
	if (sp->mode.x == exploration::modeVelocity &&
	    sp->mode.y == exploration::modeVelocity &&
	    sp->mode.z == exploration::modeVelocity) {
		spdlog::debug("take-off/land-relative");
		cf->m_pcPropellers->SetRelativePosition(
		    argos::CVector3(static_cast<double>(sp->velocity.x),
		                    static_cast<double>(sp->velocity.y),
		                    static_cast<double>(sp->velocity.z)));
	}

	/* hover */
	if (sp->mode.x == exploration::modeVelocity &&
	    sp->mode.y == exploration::modeVelocity &&
	    sp->mode.z == exploration::modeAbs && sp->velocity.x == 0.0F &&
	    sp->velocity.y == 0.0F && sp->position.z != 0.0F) {
		spdlog::debug("hover");
		// TODO
		const auto &position = cf->m_pcPos->GetReading().Position;
		const auto &orientation = cf->m_pcPos->GetReading().Orientation;
		CameraData cd =
		    cf->flow_deck_.getInitPositionDelta(position, orientation);
		cf->m_pcPropellers->SetAbsolutePosition(argos::CVector3(
		    static_cast<double>(cd.delta_x), static_cast<double>(cd.delta_y),
		    static_cast<double>(cd.z)));
	} else if (sp->mode.x == exploration::modeVelocity &&
	           sp->mode.y == exploration::modeVelocity &&
	           sp->mode.z == exploration::modeAbs) {
		spdlog::debug("velocity_command: {}", sp->velocity.x);
		auto x = sp->velocity.x / 16;
		auto y = sp->velocity.y / 16;

		cf->m_pcPropellers->SetRelativePosition(
		    argos::CVector3(static_cast<double>(x), static_cast<double>(y),
		                    static_cast<double>(0)));
	}

	/* shut_off_engines */
	if (sp->mode.x == exploration::modeDisable &&
	    sp->mode.y == exploration::modeDisable &&
	    sp->mode.z == exploration::modeDisable) {
		spdlog::debug("mode_disable");
	}
}

std::uint64_t DroneLayer::config_block_radio_address() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	return 0xE7E7E7E700U | cf->id_;
}

std::uint8_t DroneLayer::deck_bc_multiranger() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	return static_cast<uint8_t>(cf->m_pcDistance != nullptr);
}

std::uint8_t DroneLayer::deck_bc_flow2() { return 1U; }

std::uint8_t DroneLayer::radio_rssi() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	return uint8_t(cf->rssi_beacon_.read_value());
}

std::float_t DroneLayer::kalman_state_z() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	const auto &position = cf->m_pcPos->GetReading().Position;
	const auto &orientation = cf->m_pcPos->GetReading().Orientation;
	CameraData cd = cf->flow_deck_.getInitPositionDelta(position, orientation);
	return cd.z;
}

std::float_t DroneLayer::stabilizer_yaw() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	const auto &position = cf->m_pcPos->GetReading().Position;
	const auto &orientation = cf->m_pcPos->GetReading().Orientation;
	CameraData cd = cf->flow_deck_.getInitPositionDelta(position, orientation);
	float_t yaw_rad = cd.yaw;
	return static_cast<float_t>(yaw_rad * static_cast<float_t>(180) /
	                            static_cast<float_t>(PI));
}

float get_distance_sensor(CCrazyflieSensing *cf, uint8_t sensor_index) {
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
	auto sDistRead = cf->m_pcDistance->GetReadingsMap();
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

std::float_t DroneLayer::range_front() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	auto val = std::min(1.0F, get_distance_sensor(cf, 1) / 100);
	spdlog::debug("f: {}", val);
	return val;
}

std::float_t DroneLayer::range_left() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	auto val = std::min(1.0F, get_distance_sensor(cf, 0) / 100);
	spdlog::debug("l: {}", val);
	return val;
}

std::float_t DroneLayer::range_back() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	auto val = std::min(1.0F, get_distance_sensor(cf, 2) / 100);
	spdlog::debug("b: {}", val);
	return val;
}

std::float_t DroneLayer::range_right() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	auto val = std::min(1.0F, get_distance_sensor(cf, 3) / 100);
	spdlog::debug("r: {}", val);
	return val;
}

std::float_t DroneLayer::range_up() {
	// TODO Up sensor not implemented yet"
	return 1.0;
}

} // namespace porting

// NOLINTNEXTLINE
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")
