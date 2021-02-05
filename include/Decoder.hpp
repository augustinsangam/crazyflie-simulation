#include "Conn.hpp"
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

enum cmd_t : int_fast8_t { unknown = -1, none, pulse, take_off, land };

class Decoder {
private:
	const std::unordered_map<std::string, cmd_t> map_;

public:
	static std::string cmd_to_cstr(cmd_t cmd) {
		if (cmd == cmd_t::unknown) {
			return "unknown";
		}

		std::vector<std::string> types_{"", "pulse", "take_off", "land"};

		return types_[static_cast<size_t>(cmd)]; // NOLINT
	}

	Decoder();

	cmd_t decode(conn::msg_t msg);
};
