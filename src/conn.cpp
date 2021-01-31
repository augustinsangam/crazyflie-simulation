#define RAPIDJSON_HAS_STDSTRING 1

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

extern "C" {
/* ::htons */
#include <arpa/inet.h>
/*
 * ::gai_strerror
 * ::getaddrinfo
 * struct addrinfo
 */
#include <netdb.h>
/*
 * ::connect
 * ::recv
 * ::send
 * ::socket
 * struct sockaddr
 * struct sockaddr_in
 */
#include <sys/socket.h>
/* ::close */
#include <unistd.h>
}

namespace rj = rapidjson;

enum conn : int_fast8_t {
	OK = 0,
	UNCONNECTED = -1,
	UNKNOWN = -127,
};

class Conn {
	bool connected_{};
	int sock_;
	struct sockaddr_in addr_;

public:
	Conn(const std::string &host, uint16_t port)
	    : sock_{::socket(AF_INET, SOCK_STREAM, 0)}, addr_{.sin_family = AF_INET,
	                                                      .sin_port =
	                                                          ::htons(port)} {
		struct addrinfo *it, *res,
		    hints{.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};

		if (sock_ < 0) {
			throw std::runtime_error("socket() failed");
		}

		const auto status = ::getaddrinfo(host.c_str(), nullptr, &hints, &res);
		if (status != 0) {
			std::string err("invalid address: ");
			err += ::gai_strerror(status);
			throw std::runtime_error(err);
		}

		for (it = res; it != nullptr; it = it->ai_next) {
			if (it->ai_family == AF_INET) {
				break;
			}
		}

		if (it == nullptr) {
			throw std::runtime_error("address info not found");
		}

		auto *addr_in = reinterpret_cast<struct sockaddr_in *>(it->ai_addr);
		addr_.sin_addr.s_addr = addr_in->sin_addr.s_addr;

		::freeaddrinfo(res);
	}

	~Conn() { ::close(sock_); }

	std::future<conn> connect() {
		return std::async(std::launch::async, [&]() {
			const auto *addr = reinterpret_cast<const sockaddr *>(&addr_);
			const auto status = ::connect(sock_, addr, sizeof *addr);
			connected_ = status >= 0;
			return status >= 0 ? conn::OK : conn::UNKNOWN;
		});
	}

	std::future<conn> send(const std::string &s) const {
		assert(connected_); // NOLINT
		auto v = std::make_unique<std::vector<char>>(s.begin(), s.end());
		return send(std::move(v));
	}

	std::future<conn> send(const std::pair<const char *, size_t> &buf) const {
		auto v = std::make_unique<std::vector<char>>(buf.first,
		                                             buf.first + buf.second);
		return send(std::move(v));
	}

	std::future<conn> send(std::unique_ptr<std::vector<char>> v) const {
		assert(connected_); // NOLINT
		return std::async([&, v = std::move(v)]() {
			const auto wc = ::send(sock_, v->data(), v->size(), 0);
			return wc >= 0 ? conn::OK : conn::UNKNOWN;
		});
	}

	std::future<std::pair<char *, ssize_t>> recv() const {
		assert(connected_);          // NOLINT
		auto *buf = new char[65536]; // NOLINT
		return std::async([&, buf]() {
			const auto rc = ::recv(sock_, buf, sizeof buf, 0);
			return std::make_pair(buf, rc);
		});
	}
};

int main() {
	Conn conn("localhost", 3995);
	auto f1 = conn.connect();
	const auto v1 = f1.get();
	std::cout << v1 << std::endl;

	auto f2 = conn.recv();
	const auto v2 = f2.get();
	std::cout << v2.first << std::endl;
	delete[] v2.first; // NOLINT

	rj::StringBuffer sb;
	rj::Writer<rj::StringBuffer> w(sb);

	w.StartObject();
	w.String("key");
	w.String(std::string("value"));
	w.EndObject();

	auto f3 = conn.send(std::make_pair(sb.GetString(), sb.GetSize()));

	sb.Clear();
	w.Reset(sb);

	const auto v3 = f3.get();
	std::cout << v3 << std::endl;

	return 0;
}
