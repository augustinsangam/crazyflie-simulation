#include "CameraData.hpp"
#include "Conn.hpp"
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
	Vec4 pos_;
	SensorData sensors_;
	CameraData camera_;

public:
	explicit RTStatus(std::string name);

	std::string encode();

	const std::string &get_name() const { return name_; }

	void setSensorData(uint16_t left, uint16_t right, uint16_t front,
	                   uint16_t back, uint16_t up);

	void setCameraData(uint16_t delta_x_l, uint16_t delta_x_h,
	                   uint16_t delta_y_l, uint16_t delta_y_h);

	bool isFlying() const { return flying_; }

	std::string getName();

	void setPosition(Vec4 pos) { pos_ = pos; };

	void setBattery(std::float_t battery) { battery_ = battery; };

	void enable();

	void disable();

	void print() const;
};
