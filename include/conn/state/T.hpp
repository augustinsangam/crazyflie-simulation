#ifndef CONN_STATE_T_HPP
#define CONN_STATE_T_HPP

#include <cstdint>

namespace conn::state {
enum T : std::int_fast8_t {
	unknown = -127,
	plugable = 0,
	connectable,
	connected,
	disconnectable,
	unplugable,
	terminated,
};
} // namespace conn::state

#endif /* CONN_STATE_T_HPP */
