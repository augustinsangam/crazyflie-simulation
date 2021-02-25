#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace cmd {
enum T : int_fast8_t {
	unknown = -1,
	none,
	pulse,
	take_off,
	land,
	lighten,
	darken,
	start_mission
};
} // namespace cmd

class Decoder {
private:
	const std::unordered_map<std::string, cmd::T> map_{
	    {"", cmd::none},
	    {"pulse", cmd::pulse},
	    {"takeOff", cmd::take_off},
	    {"land", cmd::land},
	    {"lighten", cmd::lighten},
	    {"darken", cmd::darken},
	    {"startMission", cmd::start_mission}};

public:
	static std::string cmdo_cstr(cmd::T cmd) {
		if (cmd == cmd::unknown) {
			return "unknown";
		}

		std::vector<std::string> types_{
		    "",        "pulse",  "takeOff",     "land",
		    "lighten", "darken", "startMission"};

		return types_[static_cast<size_t>(cmd)]; // NOLINT
	}

	Decoder() = default;

	std::optional<cmd::T>
	decode(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg);
};
