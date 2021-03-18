#ifndef CONN_STATE_T_HPP
#define CONN_STATE_T_HPP

#include <cstdint>

namespace conn::state {
enum T : std::int_fast8_t {
	unknown = -127,
	init = 0,
	plugable,
	connectable,
	connected,
	disconnectable,
	unplugable,
	terminated,
};
} // namespace conn::state

#endif /* CONN_STATE_T_HPP */
