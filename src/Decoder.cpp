#include "Decoder.hpp"
#include "Conn.hpp"

#include <rapidjson/document.h>

Decoder::Decoder()
    : map_{{"", cmd_t::none},
           {"pulse", cmd_t::pulse},
           {"takeOff", cmd_t::take_off},
           {"land", cmd_t::land},
           {"lighten", cmd_t::lighten},
           {"darken", cmd_t::darken},
           {"startMission", cmd_t::start_mission}} {}

cmd_t Decoder::decode(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg) {
	rapidjson::Document document;
	document.Parse<0>(msg.first.get(), msg.second);

	const std::string msg_type = document["type"].GetString();

	const auto it = map_.find(msg_type);
	return it != map_.end() ? it->second : cmd_t::unknown;
}
