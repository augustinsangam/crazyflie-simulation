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

	static constexpr const uint8_t tick_rate_{128};

	static constexpr const uint8_t tick_tcp_send_{64};
	static constexpr const uint8_t tick_tcp_recv_{65};
	static constexpr const uint8_t tick_p2p_handler_{66};

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

	argos::CVector3 target_position_{};
	argos::CRadians target_orientation_{};
	bool stabilize_{false};

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
		auto *p = new exploration::P2PPacket{};
		p->size = 0xff;
		p->rssi = 0x80;
		// p->port = 0x63;
		p->data[0] = 0x63;
		p->data[1] = 0x01;
		sm_.p2p_callback_handler(p);
		rt_status_.enable();
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
		argos::CRadians yaw, y_angle, x_angle;
		orientation.ToEulerAngles(yaw, y_angle, x_angle);

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

		spdlog::debug("{} -> (x: {}, y: {}, z: {}, yaw: {})", id_,
		              position.GetX(), position.GetY(), position.GetZ(),
		              yaw.GetValue());
		// Update drone status
		Vec4 position_vec4 = Vec4(static_cast<std::float_t>(position.GetX()),
		                          static_cast<std::float_t>(position.GetY()),
		                          static_cast<std::float_t>(position.GetZ()));
		rt_status_.update(static_cast<std::float_t>(battery), position_vec4);

		/* ############## Update actuators and communication ############## */

		switch (tick_count_ % tick_rate_) {
		case 0:
			current_operation_ = tcp_socket_communication_send;
			break;
		case 1:
			current_operation_ = tcp_socket_communication_recv;
			break;
		case 2:
			current_operation_ = peer_to_peer_communication;
			break;
		default:
			current_operation_ = state_machine;
			break;
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
			if (!stabilize_) {
				sm_.step();
				break;
			}
			double threshold_position = 0.05;
			double threshold_orientation = 0.02;
			if (std::abs(position.GetX() - target_position_.GetX()) <
			        threshold_position &&
			    std::abs(position.GetY() - target_position_.GetY()) <
			        threshold_position &&
			    std::abs(position.GetZ() - target_position_.GetZ()) <
			        threshold_position) {
				// std::abs(yaw.GetValue() - target_orientation_.GetValue()) <
				//     threshold_orientation) {

				// std::remainder((yaw.GetValue()), 2 * PI) -
				//         std::remainder(target_orientation_.GetValue(), 2 *
				//         PI) < :0.3614741770546977 - 0.36147417705475815,
				//     threshold_orientation)
				stabilize_ = false;
			}
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

	spdlog::debug(
	    "Setpoint:\nx: ({}, {}), \ny: ({}, {}), \nz: ({}, {}) \nyaw: ({}, {})",
	    sp->mode.x, sp->velocity.x, sp->mode.y, sp->velocity.y, sp->mode.z,
	    sp->velocity.z, sp->mode.yaw, sp->attitudeRate.yaw * PI / 180);

	const auto &position = cf->m_pcPos->GetReading().Position;
	const auto &orientation = cf->m_pcPos->GetReading().Orientation;
	argos::CRadians yaw, y_angle, x_angle;
	orientation.ToEulerAngles(yaw, y_angle, x_angle);
	argos::CVector3 new_position =
	    argos::CVector3(position.GetX(), position.GetY(), position.GetZ());
	argos::CRadians new_orientation = yaw;

	if (sp->mode.x == exploration::modeDisable &&
	    sp->mode.y == exploration::modeDisable &&
	    sp->mode.z == exploration::modeDisable &&
	    sp->mode.yaw == exploration::modeDisable) {
		new_position.SetZ(0);
		new_orientation = yaw;
	}

	if (sp->mode.x == exploration::modeDisable) {
		// do nothing
	} else if (sp->mode.x == exploration::modeVelocity) {
		auto x = sp->velocity.x;
		new_position.SetX(new_position.GetX() + x);
	} else if (sp->mode.x == exploration::modeAbs) {
		spdlog::error("this should never happen ! (sp->mode.x = modeAbs)");
		// does not happen
	}

	if (sp->mode.y == exploration::modeDisable) {
		// do nothing
	} else if (sp->mode.y == exploration::modeVelocity) {
		auto y = sp->velocity.y;
		new_position.SetY(new_position.GetY() + y);
	} else if (sp->mode.y == exploration::modeAbs) {
		spdlog::error("this should never happen ! (sp->mode.y = modeAbs)");
		// does not happen
	}

	if (sp->mode.z == exploration::modeDisable) {
		new_position.SetZ(0);
	} else if (sp->mode.z == exploration::modeVelocity) {
		auto z = sp->velocity.z;
		new_position.SetZ(new_position.GetZ() + z);
	} else if (sp->mode.z == exploration::modeAbs) {
		new_position.SetZ(sp->position.z);
	}

	auto angle = sp->attitudeRate.yaw * PI / 180;
	if (sp->mode.yaw == exploration::modeDisable) {
		// do nothing
	} else if (sp->mode.yaw == exploration::modeVelocity) {
		new_orientation += argos::CRadians(angle);
	} else if (sp->mode.yaw == exploration::modeAbs) {
		spdlog::error("this should never happen ! (sp->mode.yaw = modeAbs)");
		// does not happen
	}

	if (sp->mode.x == exploration::modeVelocity &&
	    sp->mode.y == exploration::modeVelocity && sp->velocity.x == 0 &&
	    sp->velocity.y == 0 && position.GetZ() >= 0.28) {
		cf->stabilize_ = true;
		cf->target_position_ = argos::CVector3(
		    position.GetX() - 0.02, position.GetY(), position.GetZ());
		new_position = position;

		// cf->target_orientation_ = argos::CRadians(yaw.GetValue() + angle);
		// new_orientation = argos::CRadians(yaw.GetValue() + angle);
		cf->target_orientation_ = yaw;
		new_orientation = yaw;

		spdlog::info("STABILIZE: x:{}, y:{}, z:{}, yaw:{}",
		             cf->target_position_.GetX(), cf->target_position_.GetY(),
		             cf->target_position_.GetZ(),
		             cf->target_orientation_.GetValue());
	}

	cf->m_pcPropellers->SetAbsolutePosition(new_position);
	cf->m_pcPropellers->SetAbsoluteYaw(new_orientation);
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

double get_distance_sensor(CCrazyflieSensing *cf, uint8_t sensor_index) {
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
		return iterDistRead->second < 0
		           ? 1
		           : std::min(1.0, iterDistRead->second / 100);
	}
	double default_value = 0.0;
	spdlog::error("Unable to read from the multiranger sensors! Returning a "
	              "default value (" +
	              std::to_string(default_value) + ").");
	return default_value;
}

std::float_t DroneLayer::range_front() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	auto val = get_distance_sensor(cf, 1);
	spdlog::debug("f: {}", val);
	return val;
}

std::float_t DroneLayer::range_left() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	auto val = get_distance_sensor(cf, 2);
	spdlog::debug("l: {}", val);
	return val;
}

std::float_t DroneLayer::range_back() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	auto val = get_distance_sensor(cf, 3);
	spdlog::debug("b: {}", val);
	return val;
}

std::float_t DroneLayer::range_right() {
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
	auto val = get_distance_sensor(cf, 0);
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
