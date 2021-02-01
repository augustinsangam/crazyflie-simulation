#define TICKRATE 10

#include "RTStatus.hpp"
#include "conn.hpp"
#include <argos3/core/control_interface/ci_controller.h>
#include <argos3/core/utility/configuration/argos_configuration.h>
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/core/utility/math/rng.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
#include <iostream>
#include <ostream>
#include <random>

namespace uuid {
static std::random_device rd;
static std::mt19937 gen(rd());
static std::uniform_int_distribution<> dis(0, 15);
static std::uniform_int_distribution<> dis2(8, 11);

std::string generate_uuid_v4() {
	std::stringstream ss;
	int i;
	ss << std::hex;
	for (i = 0; i < 8; i++) {
		ss << dis(gen);
	}
	ss << "-";
	for (i = 0; i < 4; i++) {
		ss << dis(gen);
	}
	ss << "-4";
	for (i = 0; i < 3; i++) {
		ss << dis(gen);
	}
	ss << "-";
	ss << dis2(gen);
	for (i = 0; i < 3; i++) {
		ss << dis(gen);
	}
	ss << "-";
	for (i = 0; i < 12; i++) {
		ss << dis(gen);
	};
	return ss.str();
}
} // namespace uuid

class CCrazyflieSensing : public argos::CCI_Controller {
private:
	/* Pointer to the position actuator */
	argos::CCI_QuadRotorPositionActuator *m_pcPropellers;

	/* Pointer to the positioning sensor */
	argos::CCI_PositioningSensor *m_pcPos;

	/* Pointer to the battery sensor */
	argos::CCI_BatterySensor *m_pcBattery;

	/* The random number generator */
	argos::CRandom::CRNG *m_pcRNG;

	RTStatus rtStatus;

	Conn connection;

	uint sendFrequency = 2; // Hertz
	uint counter = TICKRATE;

	/* Current step */
	uint m_uiCurrentStep;

public:
	/* Class constructor. */
	CCrazyflieSensing()
	    : m_pcPropellers(NULL), m_pcRNG(NULL), m_pcPos(NULL), m_pcBattery(NULL),
	      m_uiCurrentStep(0), connection("localhost", 3995),
	      rtStatus(uuid::generate_uuid_v4()) {}

	/* Class destructor. */
	~CCrazyflieSensing() override = default;

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_diffusion_controller> section.
	 */
	void Init(argos::TConfigurationNode &t_node) override {
		auto f1 = connection.connect();
		const auto v1 = f1.get();
		std::cout << v1 << std::endl;
		try {
			/*
			 * Initialize sensors/actuators
			 */
			m_pcPropellers = GetActuator<argos::CCI_QuadRotorPositionActuator>(
			    "quadrotor_position");
			try {
				m_pcPos =
				    GetSensor<argos::CCI_PositioningSensor>("positioning");
			} catch (argos::CARGoSException &ex) {
			}
			try {
				m_pcBattery = GetSensor<argos::CCI_BatterySensor>("battery");
			} catch (argos::CARGoSException &ex) {
			}
		} catch (argos::CARGoSException &ex) {
		}
		/*
		 * Initialize other stuff
		 */
		/* Create a random number generator. We use the 'argos' category so
		   that creation, reset, seeding and cleanup are managed by ARGoS. */
		m_pcRNG = argos::CRandom::CreateRNG("argos");

		m_uiCurrentStep = 0;
		std::cout << "Init OK" << std::endl;
		Reset();
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
		rtStatus.update(sBatRead.AvailableCharge,
		                Vec4(cPos.GetX(), cPos.GetY(), cPos.GetZ()));

		if (--counter < TICKRATE / sendFrequency) {
			counter = TICKRATE;
			std::string json = rtStatus.encode();
			// std::cout << json << std::endl;
			connection.send(json);
		}

		m_uiCurrentStep++;
	}

	/*
	 * This function resets the controller to its state right after the
	 * Init().
	 * It is called when you press the reset button in the GUI.
	 * In this example controller there is no need for resetting anything,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	void Reset() override {}

	/*
	 * Called to cleanup what done by Init() when the experiment finishes.
	 * In this example controller there is no need for clean anything up,
	 * so the function could have been omitted. It's here just for
	 * completeness.
	 */
	void Destroy() override {}

	bool TakeOff() {
		argos::CVector3 cPos = m_pcPos->GetReading().Position;
		if (argos::Abs(cPos.GetZ() - 2.0f) < 0.01f) {
			return false;
		}
		cPos.SetZ(2.0f);
		m_pcPropellers->SetAbsolutePosition(cPos);
		return true;
	}

	bool Land() {
		argos::CVector3 cPos = m_pcPos->GetReading().Position;
		if (argos::Abs(cPos.GetZ()) < 0.01) {
			return false;
		}

		cPos.SetZ(0);
		m_pcPropellers->SetAbsolutePosition(cPos);
		return true;
	}
};

// NOLINTNEXTLINE
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")
