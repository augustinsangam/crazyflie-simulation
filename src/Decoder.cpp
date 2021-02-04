#include "Decoder.hpp"
#include "Conn.hpp"
#include <array>
#include <iostream>
#include <string>
#include <unordered_map>

#include <rapidjson/document.h>

static constexpr std::array<const char *, 4> types{"", "robot_update",
                                                   "take_off", "land"};

static std::unordered_map<std::string, cmd_t> map_; // NOLINT

static const std::unordered_map<std::string, cmd_t> &map() {
	if (map_.empty()) {
		const auto len = types.size();
		for (size_t i = 0; i < len; ++i) {
			map_[std::string(types[i])] = static_cast<cmd_t>(i); // NOLINT
		}
	}
	return map_;
}

cmd_t Decoder::decode(conn::msg_t msg) {
	rapidjson::Document document;
	document.Parse<0>(msg.first, msg.second);

	const std::string msg_type = document["type"].GetString();

	const auto it = map().find(msg_type);
	return it != map().end() ? it->second : cmd_t::unknown;
}
