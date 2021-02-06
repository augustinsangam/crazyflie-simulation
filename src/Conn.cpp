#include "Conn.hpp"

#include <cassert>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <utility>

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

#include <iostream>

namespace conn {

void Conn::input_thread(Conn *conn) {
	{
		std::unique_lock<std::mutex> u_lock(conn->connexion_mtx_);
		conn->connexion_wait_.wait(u_lock,
		                           [conn]() { return conn->connected_; });
	}

	bool input_chn_open;
	do {
		auto *mem = new char[conn->msg_len_];
		const auto rc = ::recv(conn->sock_, mem, conn->msg_len_, 0);
		auto msg = make_pair(std::unique_ptr<char[]>(mem), rc);

		if (rc < 0) {
			std::cerr << "input_thread: recv failed with " << rc << std::endl;
			break;
		}

		input_chn_open = conn->input_chn_.send(std::move(msg));
	} while (input_chn_open);

	std::cout << "input_thread: done" << std::endl;
}

void Conn::output_thread(Conn *conn) {
	{
		std::unique_lock<std::mutex> u_lock(conn->connexion_mtx_);
		conn->connexion_wait_.wait(u_lock,
		                           [conn]() { return conn->connected_; });
	}

	for (;;) {
		const auto msg = conn->output_chn_.recv(true);
		if (!msg) {
			std::cerr << "output_thread: msg nul" << std::endl;
			break;
		}

		const auto wc = ::send(conn->sock_, msg->data(), msg->size(), 0);
		if (wc < 0) {
			std::cerr << "output_thread: send failed with " << wc << std::endl;
			break;
		}
	}
}

Conn::Conn(const std::string &host, uint16_t port, std::size_t msg_len)
    : msg_len_{msg_len}, sock_{::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)},
      addr_{.sin_family = AF_INET, .sin_port = ::htons(port)},
      input_thr_(input_thread, this), output_thr_(output_thread, this) {
	struct addrinfo *it, *res,
	    hints{.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};

	std::cout << "sock: " << sock_ << std::endl;
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

Conn::~Conn() { disconnect(); }

state Conn::connect() {
	const auto *addr = reinterpret_cast<const sockaddr *>(&addr_);
	const auto status = ::connect(sock_, addr, sizeof *addr);
	connected_ = status >= 0;
	if (connected_) {
		connexion_wait_.notify_all();
		// FIXME:
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	return connected_ ? conn::ok : conn::unknown;
}

void Conn::disconnect() {
	if (connected_) {
		// connected_ = false;
		//::close(sock_);
	}
}

void Conn::send(std::string &&msg) { output_chn_.send(std::move(msg)); }

std::optional<std::pair<std::unique_ptr<char[]>, std::size_t>> Conn::recv() {
	return input_chn_.recv();
}

} // namespace conn
