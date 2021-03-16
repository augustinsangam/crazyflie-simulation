#ifndef RADIO_LINK_HPP
#define RADIO_LINK_HPP

#define P2P_MAX_DATA_SIZE 60

#include <stdint.h>

// NOLINTNEXTLINE
typedef struct _P2PPacket {
	uint8_t size; //< Size of data
	uint8_t rssi; //< Received Signal Strength Intensity
	union {
		struct {
			uint8_t port; //< Header selecting channel and port
			uint8_t data[P2P_MAX_DATA_SIZE]; //< Data
		};
		uint8_t raw[P2P_MAX_DATA_SIZE + 1]; //< The full packet "raw"
	};
} __attribute__((packed)) P2PPacket;

#endif /* RADIO_LINK_HPP */
