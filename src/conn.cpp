#include "conn.hpp"

#include <cassert>
#include <functional>
#include <memory>
#include <stdexcept>

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
 */
#include <sys/socket.h>
/* ::close */
#include <unistd.h>
}

Conn::Conn(const std::string &host, uint16_t port)
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

Conn::~Conn() { ::close(sock_); }

std::future<conn> Conn::connect() {
	return std::async(std::launch::async, [&]() {
		const auto *addr = reinterpret_cast<const sockaddr *>(&addr_);
		const auto status = ::connect(sock_, addr, sizeof *addr);
		connected_ = status >= 0;
		return status >= 0 ? conn::OK : conn::UNKNOWN;
	});
}

std::future<conn> Conn::send(const std::string &s) const {
	auto v = std::make_unique<std::vector<char>>(s.begin(), s.end());
	return send(std::move(v));
}

std::future<conn> Conn::send(const std::pair<const char *, size_t> &buf) const {
	auto v =
	    std::make_unique<std::vector<char>>(buf.first, buf.first + buf.second);
	return send(std::move(v));
}

std::future<conn> Conn::send(std::unique_ptr<std::vector<char>> v) const {
	assert(connected_); // NOLINT
	return std::async([&, v = std::move(v)]() {
		const auto wc = ::send(sock_, v->data(), v->size(), 0);
		return wc >= 0 ? conn::OK : conn::UNKNOWN;
	});
}

std::future<std::pair<char *, ssize_t>> Conn::recv() const {
	assert(connected_);          // NOLINT
	auto *buf = new char[65536]; // NOLINT
	return std::async([&, buf]() {
		const auto rc = ::recv(sock_, buf, sizeof buf, 0);
		return std::make_pair(buf, rc);
	});
}
