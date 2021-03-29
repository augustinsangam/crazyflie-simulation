#ifndef CMD_T_HPP
#define CMD_T_HPP

#include <cstdint>

namespace cmd {
enum T : std::int_fast8_t {
	unknown = -1,
	none,
	pulse,
	start_mission,
	land,
	lighten,
	return_to_base,
	darken,
	take_off,
};
} // namespace cmd

#endif /* CMD_T_HPP */
