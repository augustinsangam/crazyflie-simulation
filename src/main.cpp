#include <argos3/core/control_interface/ci_controller.h>
/* Definition of the crazyflie distance sensor */
#include <argos3/plugins/robots/crazyflie/control_interface/ci_crazyflie_distance_scanner_sensor.h>
/* Definition of the crazyflie position actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_quadrotor_position_actuator.h>
/* Definition of the crazyflie position sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_positioning_sensor.h>
/* Definition of the crazyflie range and bearing actuator */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_actuator.h>
/* Definition of the crazyflie range and bearing sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_range_and_bearing_sensor.h>
/* Definition of the crazyflie battery sensor */
#include <argos3/plugins/robots/generic/control_interface/ci_battery_sensor.h>
/* Definitions for random number generation */
#include <argos3/core/utility/math/rng.h>
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
/* Logging */
#include <argos3/core/utility/logging/argos_log.h>
#include <argos3/plugins/robots/generic/control_interface/ci_leds_actuator.h>

class CCrazyflieSensing : public argos::CCI_Controller {
private:
	/* Pointer to the crazyflie distance sensor */
	argos::CCI_CrazyflieDistanceScannerSensor *m_pcDistance;

	/* Pointer to the position actuator */
	argos::CCI_QuadRotorPositionActuator *m_pcPropellers;

	/* Pointer to the range and bearing actuator */
	argos::CCI_RangeAndBearingActuator *m_pcRABA;

	/* Pointer to the range and bearing sensor */
	argos::CCI_RangeAndBearingSensor *m_pcRABS;

	/* Pointer to the positioning sensor */
	argos::CCI_PositioningSensor *m_pcPos;

	/* Pointer to the battery sensor */
	argos::CCI_BatterySensor *m_pcBattery;

	/* The random number generator */
	argos::CRandom::CRNG *m_pcRNG;

	/* Current step */
	uint m_uiCurrentStep;

public:
	/* Class constructor. */
	CCrazyflieSensing()
	    : m_pcDistance(NULL), m_pcPropellers(NULL), m_pcRNG(NULL),
	      m_pcRABA(NULL), m_pcRABS(NULL), m_pcPos(NULL), m_pcBattery(NULL),
	      m_uiCurrentStep(0) {}

	/* Class destructor. */
	~CCrazyflieSensing() override = default;

	/*
	 * This function initializes the controller.
	 * The 't_node' variable points to the <parameters> section in the XML
	 * file in the <controllers><footbot_diffusion_controller> section.
	 */
	void Init(argos::TConfigurationNode &t_node) override {
		try {
			/*
			 * Initialize sensors/actuators
			 */
			m_pcDistance = GetSensor<argos::CCI_CrazyflieDistanceScannerSensor>(
			    "crazyflie_distance_scanner");
			m_pcPropellers = GetActuator<argos::CCI_QuadRotorPositionActuator>(
			    "quadrotor_position");
			/* Get pointers to devices */
			m_pcRABA = GetActuator<argos::CCI_RangeAndBearingActuator>(
			    "range_and_bearing");
			m_pcRABS = GetSensor<argos::CCI_RangeAndBearingSensor>(
			    "range_and_bearing");
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
		Reset();
	}

	/*
	 * This function is called once every time step.
	 * The length of the time step is set in the XML file.
	 */
	void ControlStep() override {
		/*if (--cnt_ == 0) {
		    cnt_ = 10;
		    m_pcLEDs_->SetAllColors(led_state_ ? argos::CColor::BLACK
		                                       : argos::CColor::RED);
		    led_state_ = !led_state_;
		}*/

		m_pcPropellers->SetRelativeYaw(argos::CRadians::PI_OVER_SIX);

		// Takeoff/Land
		if ((m_uiCurrentStep / 10) % 2 == 0) {
			TakeOff();
		} else {
			Land();
		}
		// Look battery level
		const argos::CCI_BatterySensor::SReading &sBatRead =
		    m_pcBattery->GetReading();
		argos::LOG << "Battery level: " << sBatRead.AvailableCharge
		           << std::endl;

		// Look here for documentation on the distance sensor:
		// /root/argos3/src/plugins/robots/crazyflie/control_interface/ci_crazyflie_distance_scanner_sensor.h
		// Read distance sensor
		argos::CCI_CrazyflieDistanceScannerSensor::TReadingsMap sDistRead =
		    m_pcDistance->GetReadingsMap();
		auto iterDistRead = sDistRead.begin();
		if (sDistRead.size() == 4) {
			argos::LOG << "Front dist: " << (iterDistRead++)->second
			           << std::endl;
			argos::LOG << "Left dist: " << (iterDistRead++)->second
			           << std::endl;
			argos::LOG << "Back dist: " << (iterDistRead++)->second
			           << std::endl;
			argos::LOG << "Right dist: " << (iterDistRead)->second << std::endl;
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
		if (argos::Abs(cPos.GetZ()) < 0.01f)
			return false;
		cPos.SetZ(0.0f);
		m_pcPropellers->SetAbsolutePosition(cPos);
		return true;
	}
};

// NOLINTNEXTLINE
REGISTER_CONTROLLER(CCrazyflieSensing, "crazyflie_sensing_controller")
