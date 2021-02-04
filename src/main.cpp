#include "Conn.hpp"
#include "Decoder.hpp"
#include "RTStatus.hpp"
#include "UUID.hpp"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <bits/stdint-uintn.h>
#include <cstdint>
#include <iostream>
#include <ostream>
#include <random>
#include <string>
#include <thread>
#include <utility>

class CCrazyflieSensing : public argos::CCI_Controller, public conn::Callable {
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
	Decoder decoder_;
	MESSAGE_TYPE next_command_;

	/* Pointer to the position actuator */
	argos::CCI_QuadRotorPositionActuator *m_pcPropellers{};

	/* Pointer to the positioning sensor */
	argos::CCI_PositioningSensor *m_pcPos{};

	/* Pointer to the battery sensor */
	argos::CCI_BatterySensor *m_pcBattery{};

public:
	/* Class constructor. */
	CCrazyflieSensing()
	    : conn_("localhost", 3995, this), rt_status_(uuid_gen()), decoder_(),
	      next_command_(MESSAGE_TYPE::NONE) {
		std::cout << "drone " << rt_status_.get_name() << " created"
		          << std::endl;
	}

	/* Class destructor. */
	~CCrazyflieSensing() override = default;

	void call(conn::mut_msg_t msg) override {
		std::cout << "msg.second: " << msg.second << std::endl;
		next_command_ = decoder_.decode(msg);
		delete[] msg.first; // NOLINT
	}

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_diffusion_controller> section.
	 */
	void Init(argos::TConfigurationNode & /*t_node*/) override {
		const auto v1 = conn_.connect();
		std::cout << "connexion status: " << static_cast<int>(v1) << std::endl;

		m_pcPropellers = GetActuator<argos::CCI_QuadRotorPositionActuator>(
		    "quadrotor_position");

		m_pcPos = GetSensor<argos::CCI_PositioningSensor>("positioning");

		m_pcBattery = GetSensor<argos::CCI_BatterySensor>("battery");

		std::cout << "Init OK" << std::endl;

		std::thread commandReceiver(commandsReceiver, this);
		commandReceiver.detach();
	}

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	void ControlStep() override {
		m_pcPropellers->SetRelativeYaw(argos::CRadians::PI_OVER_SIX);

		// Retrieve drone data
		const argos::CCI_BatterySensor::SReading &sBatRead =
		    m_pcBattery->GetReading();
		argos::CVector3 cPos = m_pcPos->GetReading().Position;

		// Update drone status
		rt_status_.update(static_cast<std::float_t>(sBatRead.AvailableCharge),
		                  Vec4(static_cast<std::float_t>(cPos.GetX()),
		                       static_cast<std::float_t>(cPos.GetY()),
		                       static_cast<std::float_t>(cPos.GetZ())));

		if (tick_count_ % tick_pulse_ == 0) {
			// std::string json = rt_status_.encode();
			// std::cout << json << std::endl;
			conn_.send(rt_status_.encode());
		}
		switch (next_command_) {
		case MESSAGE_TYPE::TAKEOFF:
			TakeOff();
			break;
		case MESSAGE_TYPE::LAND:
			Land();
			break;
		default:
			break;
		}
		next_command_ = MESSAGE_TYPE::NONE;
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

	bool TakeOff() {
		std::cout << "TAKEOFF" << std::endl;

		argos::CVector3 cPos = m_pcPos->GetReading().Position;
		if (argos::Abs(cPos.GetZ() - 2) < 0.01) {
			return false;
		}

		rt_status_.enable();

		cPos.SetZ(2);
		m_pcPropellers->SetAbsolutePosition(cPos);
		return true;
	}

	bool Land() {
		std::cout << "LAND" << std::endl;

		argos::CVector3 cPos = m_pcPos->GetReading().Position;
		if (argos::Abs(cPos.GetZ()) < 0.01) {
			return false;
		}

		rt_status_.disable();

		cPos.SetZ(0);
		m_pcPropellers->SetAbsolutePosition(cPos);
		return true;
	}

	static void commandsReceiver(CCrazyflieSensing *ccfls) {
		/*bool ok;
		do {
		    auto fut = ccfls->conn_.recv();
		    std::cout << "Listening..." << std::endl;

		    const auto msg = fut.get();
		    std::cout << "Message len: " << msg.second << std::endl;
		    ok = msg.second > 0;

		    if (ok) {
		        std::string msgstr(msg.first, msg.first + msg.second);
		        std::cout << msgstr << std::endl;
		        ccfls->next_command_ = ccfls->decoder_.decode(msgstr);
		    }
		    delete[] msg.first; // NOLINT
		} while (ok);
		ccfls->conn_.disconnect();*/
	}
};

// NOLINTNEXTLINE
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")
