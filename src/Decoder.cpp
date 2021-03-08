#include "Decoder.hpp"

#include <tao/json/from_string.hpp>

std::optional<cmd::T>
Decoder::decode(std::pair<std::unique_ptr<char[]>, std::size_t> &&msg) {
	const auto v = tao::json::from_string(msg.first.get(), msg.second);

	const auto &m = v.get_object();

	const auto &t = m.at("type").get_string();

	const auto it = map_.find(t);
	return it != map_.end() ? it->second : cmd::unknown;
}
