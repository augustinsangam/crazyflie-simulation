#include "porting.hpp"
#include "CCrazyflieSensing.hpp"

namespace porting {

std::uint64_t timestamp_us() {
	return 0; // TODO()
}

void Porting::kalman_estimated_pos(exploration::point_t *pos) {
	// TODO()
	// This is How to get CCrazyflieSensing Pointer
	auto *cf = reinterpret_cast<CCrazyflieSensing *>(ctx_);
}

void Porting::p2p_register_cb(void (*cb)(exploration::P2PPacket *)) {
	// TODO()
}

void Porting::radiolink_broadcast_packet(exploration::P2PPacket *packet) {
	// TODO()
}

void Porting::Porting::system_wait_start() {
	// TODO()
}

void Porting::delay_ticks(uint32_t ticks) {
	// TODO()
}

void Porting::commander_set_point(exploration::setpoint_t *sp, int prio) {
	// TODO()
}

std::uint64_t Porting::config_block_radio_address() {
	return 0; // TODO()
}

std::uint8_t Porting::deck_bc_multiranger() {
	return 0; // TODO()
}

std::uint8_t Porting::deck_bc_flow2() {
	return 0; // TODO()
}

std::uint8_t Porting::radio_rssi() {
	return 0; // TODO()
}

std::float_t Porting::kalman_state_z() {
	return 0; // TODO()
}

std::float_t Porting::stabilizer_yaw() {
	return 0; // TODO()
}

std::float_t Porting::range_front() {
	return 0; // TODO()
}

std::float_t Porting::range_left() {
	return 0; // TODO()
}

std::float_t Porting::range_back() {
	return 0; // TODO()
}

std::float_t Porting::range_right() {
	return 0; // TODO()
}

std::float_t Porting::range_up() {
	return 0; // TODO()
}

} // namespace porting
