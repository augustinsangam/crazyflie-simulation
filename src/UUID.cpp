#include "UUID.hpp"
#include <array>

extern "C" {
#include <uuid/uuid.h>
}

std::string uuid_gen() {
	uuid_t uuidObj;
	uuid_generate(uuidObj);            // NOLINT
	std::array<char, 37> buf;          // NOLINT
	uuid_unparse(uuidObj, buf.data()); // NOLINT
	return std::string(buf.data(), buf.size() - 1);
}
