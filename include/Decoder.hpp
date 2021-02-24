#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

enum cmd_t : int_fast8_t {
	unknown = -1,
	none,
	pulse,
	take_off,
	land,
	lighten,
	darken,
	start_mission
};

class Decoder {
private:
	const std::unordered_map<std::string, cmd_t> map_;

public:
	static std::string cmd_to_cstr(cmd_t cmd) {
		if (cmd == cmd_t::unknown) {
			return "unknown";
		}

		std::vector<std::string> types_{
		    "",        "pulse",  "takeOff",     "land",
		    "lighten", "darken", "startMission"};

		return types_[static_cast<size_t>(cmd)]; // NOLINT
	}

	Decoder();

	std::optional<cmd_t>
	decode(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg);
	std::string getName(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg);
	bool msgValid(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg);
};
