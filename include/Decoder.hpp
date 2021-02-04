#include "Conn.hpp"
#include <cstdint>
#include <string>

enum cmd_t : int_fast8_t { unknown = -1, none, pulse, take_off, land };

class Decoder {
public:
	static constexpr std::array<const char *, 4> types{"", "pulse", "take_off",
	                                                   "land"};

	static constexpr const char *cmd_to_cstr(cmd_t cmd) {
		if (cmd == cmd_t::unknown) {
			return "unknown";
		}

		return types[static_cast<size_t>(cmd)]; // NOLINT
	}

	static cmd_t decode(conn::msg_t msg);
};
