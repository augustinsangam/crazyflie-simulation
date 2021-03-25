#include "Vec4.hpp"
#include <cmath>

class RSSIBeacon {
private:
	Vec4 init_position_;
	float_t beam_{};

public:
	RSSIBeacon() : init_position_(0){};
	void init(Vec4 initPos);
	void update(Vec4 dronePos);
	float_t read_value() const;
};
