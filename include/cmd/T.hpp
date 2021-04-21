#ifndef CMD_T_HPP
#define CMD_T_HPP

#include <cstdint>

namespace cmd {
enum T : std::int_fast8_t {
	unknown = -1,
	none,
	pulse,
	start_mission,
	stop_mission,
	land,
	return_to_base,
	take_off,
};
} // namespace cmd

#endif /* CMD_T_HPP */
