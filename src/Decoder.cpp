#include "Decoder.hpp"

Decoder::Decoder() {
	/* must be the same order as enum in Decoder.hpp */
	types.emplace_back("UNKNOWN");
	types.emplace_back("take_off");
	types.emplace_back("land");
	types.emplace_back("robot_update");
}

void Decoder::decode(const std::string &msg) {
	rj::Document document;
	document.Parse<0>(msg.c_str());
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
	case MESSAGE_TYPE::RTUPDATE:
		decodeRTUpdate(document);
		break;
	case MESSAGE_TYPE::UNKNOWN:
		std::cout << "Unknown message received" << std::endl;
		break;
	}
};

MESSAGE_TYPE Decoder::decodeType(const std::string &msg) {
	for (size_t i = 0; i < types.size(); i++) {
		if (types[i] == msg) {
			return static_cast<MESSAGE_TYPE>(i);
		}
	}
	return MESSAGE_TYPE::UNKNOWN;
}

void Decoder::decodeRTUpdate(const rj::Document &doc) {
	const rj::Value &data = doc["data"];
	assert(data.IsArray());
	for (rapidjson::Value::ConstValueIterator itr = data.Begin();
	     itr != data.End(); ++itr) {
		const rapidjson::Value &droneRTData = *itr;
		assert(droneRTData.IsObject());
		for (rapidjson::Value::ConstMemberIterator itr2 =
		         droneRTData.MemberBegin();
		     itr2 != droneRTData.MemberEnd(); ++itr2) {
			std::cout << itr2->name.GetString() << " : "
			          << itr2->value.GetDouble() << std::endl;
		}
	}
}
