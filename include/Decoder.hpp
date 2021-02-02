#include <string>
#include <vector>

enum class MESSAGE_TYPE {
	UNKNOWN = 0,
	TAKEOFF = 1,
	LAND = 2,
	ROBOT_UPDATE = 3,
	NONE = 4
};

class Decoder {
private:
	std::vector<std::string> types;

	MESSAGE_TYPE decodeType(const std::string &msg);

public:
	Decoder();

	MESSAGE_TYPE decode(const std::string &msg);
};
