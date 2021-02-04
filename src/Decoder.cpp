#include "Decoder.hpp"
#include "Conn.hpp"
#include <iostream>
#include <rapidjson/document.h>

namespace rj = rapidjson;

Decoder::Decoder() {
	/* must be the same order as enum in Decoder.hpp */
	types.emplace_back("UNKNOWN");
	types.emplace_back("take_off");
	types.emplace_back("land");
	types.emplace_back("robot_update");
	types.emplace_back("none");
}

MESSAGE_TYPE Decoder::decode(conn::msg_t msg) {
	rj::Document document;
	std::cout << "Decoding..." << std::endl;
	document.Parse<0>(msg.first, msg.second);

	std::string msgType = document["type"].GetString();
	std::cout << msgType << std::endl;
	MESSAGE_TYPE type = decodeType(msgType);

	switch (type) {
	case MESSAGE_TYPE::TAKEOFF:
		std::cout << "TakeOff()" << std::endl;
		break;
	case MESSAGE_TYPE::LAND:
		std::cout << "Land()" << std::endl;
		break;
	case MESSAGE_TYPE::ROBOT_UPDATE:
		std::cout << "robot_update received" << std::endl;
		break;
	case MESSAGE_TYPE::UNKNOWN:
		std::cout << "Unknown message received" << std::endl;
		break;
	case MESSAGE_TYPE::NONE:
		std::cout << "None received" << std::endl;
		break;
	}
	return type;
}

MESSAGE_TYPE Decoder::decodeType(const std::string &msg) {
	for (size_t i = 0; i < types.size(); i++) {
		if (types[i] == msg) {
			return static_cast<MESSAGE_TYPE>(i);
		}
	}
	return MESSAGE_TYPE::UNKNOWN;
}
