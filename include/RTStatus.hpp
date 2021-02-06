#include "Conn.hpp"
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

public:
	explicit RTStatus(std::string name);

	std::string encode();

	const std::string &get_name() const { return name_; }

	void update(std::float_t battery, const Vec4 &pos);

	void enable();

	void disable();

	void print() const;
};
