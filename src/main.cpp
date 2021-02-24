#include "Conn.hpp"
#include "Decoder.hpp"
#include "RTStatus.hpp"
#include <argos3/core/utility/math/vector3.h>
#include <bits/stdint-uintn.h>
#include <cstdint>
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
/* argos::CCI_BatterySensor */
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
/* argos::CCI_PositioningSensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* argos::CCI_QuadRotorPositionActuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>

static uint16_t mainId = 0;
/**
 * @brief Main controller class for ARGoS simulator
 *
 */
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
	std::array<argos::CVector3, 4> squareMoves_ = {
	    argos::CVector3(1, 0, 0.5), argos::CVector3(0, 1, 0.5),
	    argos::CVector3(-1, 0, 0.5), argos::CVector3(0, -1, 0.5)};

	/* Pointer to the position actuator */
	argos::CCI_QuadRotorPositionActuator *m_pcPropellers{};

	/* Pointer to the positioning sensor */
	argos::CCI_PositioningSensor *m_pcPos{};

	/* Pointer to the battery sensor */
	argos::CCI_BatterySensor *m_pcBattery{};

public:
	/* Class constructor. */
	CCrazyflieSensing()
	    : conn_("localhost", 3995),
	      rt_status_("argos_drone_" + std::to_string(mainId)), decoder_() {
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

		std::cout << "Init OK" << std::endl;
	}

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	void ControlStep() override {

		// Retrieve drone data
		const auto &sBatRead = m_pcBattery->GetReading();
		const auto &cPos = m_pcPos->GetReading().Position;

		// Update drone status
		rt_status_.update(static_cast<std::float_t>(sBatRead.AvailableCharge),
		                  Vec4(static_cast<std::float_t>(cPos.GetX()),
		                       static_cast<std::float_t>(cPos.GetY()),
		                       static_cast<std::float_t>(cPos.GetZ())));

		switch (conn_.status()) {
		case conn::connected:
			if (tick_count_ % tick_pulse_ == 0) {
				conn_.send(rt_status_.encode());
			} else {
				auto msg = conn_.recv();
				if (msg && decoder_.msgValid(std::move(*msg))) {
					auto dest = decoder_.getName(std::move(*msg));
					auto cmd = decoder_.decode(std::move(*msg));
					if (dest == rt_status_.get_name()) {
						switch (cmd) {
						case cmd_t::take_off:
							if (!rt_status_.isFlying()) {
								std::cout
								    << rt_status_.get_name() + " : TakeOff()"
								    << std::endl;
								rt_status_.enable();
							}
							break;
						case cmd_t::land:
							if (rt_status_.isFlying()) {
								std::cout << rt_status_.get_name() + " : Land()"
								          << std::endl;
								rt_status_.disable();
							}
							break;
						case cmd_t::unknown:
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
		++tick_count_;

		if (rt_status_.isFlying()) {
			argos::CVector3 nextMove;
			++counter_;
			if (counter_ < 1 * squareSize_) {
				nextMove = squareMoves_[0];
			} else if (counter_ < 2 * squareSize_) {
				nextMove = squareMoves_[1];
			} else if (counter_ < 3 * squareSize_) {
				nextMove = squareMoves_[2];
			} else if (counter_ < 4 * squareSize_) {
				nextMove = squareMoves_[3];
			} else {
				counter_ = 0;
				m_pcPropellers->SetAbsolutePosition(
				    argos::CVector3(0, 0, cPos.GetZ()));
				return;
			}
			m_pcPropellers->SetRelativePosition(nextMove);
		} else {
			m_pcPropellers->SetAbsolutePosition(
			    argos::CVector3(0.1 * id_, 0.1 * id_, 0.01));
		}
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
