#include "rapidjson/rapidjson.h"
#include <cassert>
#include <functional>
#include <iostream>
#include <map>
#include <rapidjson/document.h>
#include <vector>

enum class MESSAGE_TYPE {
	UNKNOWN = 0,
	TAKEOFF = 1,
	LAND = 2,
	ROBOT_UPDATE = 3
};

class Decoder {
private:
	std::vector<std::string> types;
	MESSAGE_TYPE decodeType(const std::string &msg);

public:
	Decoder();
	void decode(const std::string &msg);
};
