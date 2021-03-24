#include "porting.hpp"

namespace porting {

void kalman_estimated_pos(exploration::point_t *pos) {
	// TODO()
}

std::uint64_t timestamp_us() {
	return 0; // TODO()
}

void p2p_register_cb(void (*cb)(exploration::P2PPacket *)) {
	// TODO()
}

void radiolink_broadcast_packet(exploration::P2PPacket *packet) {
	// TODO()
}

void system_wait_start() {
	// TODO()
}

void delay_ticks(uint32_t ticks) {
	// TODO()
}

void commander_set_point(exploration::setpoint_t *sp, int prio) {
	// TODO()
}

std::uint64_t config_block_radio_address() {
	return 0; // TODO()
}

std::uint8_t deck_bc_multiranger() {
	return 0; // TODO()
}

std::uint8_t deck_bc_flow2() {
	return 0; // TODO()
}

std::uint8_t radio_rssi() {
	return 0; // TODO()
}

std::float_t kalman_state_z() {
	return 0; // TODO()
}

std::float_t stabilizer_yaw() {
	return 0; // TODO()
}

std::float_t range_front() {
	return 0; // TODO()
}

std::float_t range_left() {
	return 0; // TODO()
}

std::float_t range_back() {
	return 0; // TODO()
}

std::float_t range_right() {
	return 0; // TODO()
}

std::float_t range_up() {
	return 0; // TODO()
}

} // namespace porting
