#include "Decoder.hpp"

#include <rapidjson/document.h>
#include <rapidjson/error/error.h>

Decoder::Decoder()
    : map_{{"", cmd_t::none},
           {"pulse", cmd_t::pulse},
           {"takeOff", cmd_t::take_off},
           {"land", cmd_t::land},
           {"lighten", cmd_t::lighten},
           {"darken", cmd_t::darken},
           {"startMission", cmd_t::start_mission}} {}

std::optional<cmd_t>
Decoder::decode(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg) {
	rapidjson::Document doc;
	rapidjson::ParseResult ok = doc.Parse(msg.first.get(), msg.second);
	if (!ok) {
		return std::nullopt;
	}

	const std::string msg_type = doc["type"].GetString();

	const auto it = map_.find(msg_type);
	return it != map_.end() ? it->second : cmd_t::unknown;
}

bool Decoder::msgValid(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg) {
	rapidjson::Document doc;
	rapidjson::ParseResult result = doc.Parse(msg.first.get(), msg.second);
	if (result.IsError()) {
		std::cout << "Received non parsable message" << std::endl;
	}
	return !result.IsError();
}

std::string
Decoder::getName(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg) {
	rapidjson::Document doc;
	rapidjson::ParseResult ok = doc.Parse(msg.first.get(), msg.second);
	const auto data = doc["data"].GetObject();
	const auto *name = data["name"].GetString();
	return name;
}
