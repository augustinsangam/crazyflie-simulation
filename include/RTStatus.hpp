#include "Conn.hpp"
#include "Vec4.hpp"
#include <cmath>
#include <string>

#include <rapidjson/document.h>
#include <rapidjson/pointer.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

class RTStatus {
private:
	bool flying_;
	std::string name_;
	std::float_t speed_;
	std::float_t battery_;
	Vec4 pos_;

	rapidjson::Document d_;
	rapidjson::StringBuffer sb_;
	rapidjson::Writer<rapidjson::StringBuffer> w_;

public:
	explicit RTStatus(std::string name);

	conn::msg_t encode();

	const std::string &get_name() const { return name_; }

	void update(std::float_t battery, const Vec4 &pos);

	void enable();

	void disable();

	void print() const;
};
