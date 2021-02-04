#include "Conn.hpp"
#include <cstdint>

enum cmd_t : int_fast8_t { unknown = -1, none, pulse, take_off, land };

class Decoder {
public:
	static cmd_t decode(conn::msg_t msg);
};
