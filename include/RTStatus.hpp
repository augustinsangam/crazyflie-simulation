#include "Brain.hpp"
#include "SensorData.hpp"
#include "Vec4.hpp"
#include <cmath>
#include <string>

class RTStatus {
private:
	bool flying_;
	std::string name_;
	std::float_t speed_;
	std::float_t battery_;
	SensorData sensor_data_;
	Vec4 pos_;
	float_t yaw_;
	enum PulseIndex {
		onTheGround,
		takingOff,
		landing,
		crashed,
		exploring,
		returningToBase
	};
	const std::array<std::string, 6> pulse_states = {
	    "onTheGround", "takingOff", "landing",
	    "crashed",     "exploring", "returningToBase"};
	std::string pulse_state = pulse_states[onTheGround];

public:
	explicit RTStatus(std::string name);

	std::string encode();

	const std::string &get_name() const { return name_; }

	void update(std::float_t battery, const Vec4 &pos, const float_t &yaw,
	            const SensorData &sd, const brain::State &brain_state,
	            const bool &brain_returning_to_base);

	bool isFlying() const { return flying_; }

	void setPosition(Vec4 pos) { pos_ = pos; };

	void setBattery(std::float_t battery) { battery_ = battery; };

	void enable();

	void disable();

	void print() const;

	static float_t trunc(float_t val, int numDigits);
};
