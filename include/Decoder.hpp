#include "Conn.hpp"
#include <cstdint>
#include <string>

enum cmd_t : int_fast8_t { unknown = -1, none, pulse, take_off, land };

class Decoder {
public:
	static std::string cmd_to_str(cmd_t cmd);
	static cmd_t decode(conn::msg_t msg);
};
