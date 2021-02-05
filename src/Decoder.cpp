#include "Decoder.hpp"
#include "Conn.hpp"
#include <array>
#include <iostream>
#include <string>

#include <rapidjson/document.h>

Decoder::Decoder()
    : map_{{"", cmd_t::none},
           {"pulse", cmd_t::pulse},
           {"take_off", cmd_t::take_off},
           {"land", cmd_t::land}} {}

cmd_t Decoder::decode(conn::msg_t msg) {
	rapidjson::Document document;
	document.Parse<0>(msg.first, msg.second);

	const std::string msg_type = document["type"].GetString();

	const auto it = map_.find(msg_type);
	return it != map_.end() ? it->second : cmd_t::unknown;
}
