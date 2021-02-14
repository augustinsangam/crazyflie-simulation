#include "Conn.hpp"
#include "SafePrint.hpp"
#include <asm-generic/errno.h>
#include <cassert>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
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
 * ::shutdown
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
	for (;;) {
		{
			std::unique_lock<std::mutex> u_lock(conn->connect_mtx_);
			conn->connect_wait_.wait(
			    u_lock, [conn]() { return conn->state_ == connected; });
		}

		for (;;) {
			auto *mem = new char[conn->msg_len_];
			const auto rc = ::recv(conn->sock_, mem, conn->msg_len_, 0);

			if (rc == 0) {
				conn->state_ = unplugable;
				delete[] mem;
				break;
			}

			if (rc < 0) {
				switch (errno) {
				case ECONNRESET:
					conn->state_ = unplugable;
					break;
				default:
					conn->state_ = unknown;
				}
				delete[] mem;
				break;
			}

			auto msg = make_pair(std::unique_ptr<char[]>(mem), rc);

			conn->input_chn_.send(std::move(msg));
		}
	}
}

void Conn::output_thread(Conn *conn) {
	for (;;) {
		{
			std::unique_lock<std::mutex> u_lock(conn->connect_mtx_);
			conn->connect_wait_.wait(
			    u_lock, [conn]() { return conn->state_ == connected; });
		}

		bool loop;
		do {
			const auto msg = conn->output_chn_.recv(true);

			loop = msg && ::send(conn->sock_, msg->data(), msg->size(), 0) >= 0;
		} while (loop);

		switch (errno) {
		case EPIPE:
		case ECONNRESET:
			conn->state_ = unplugable;
			break;
		default:
			conn->state_ = unknown;
		}
	}
}

Conn::Conn(const std::string &host, uint16_t port, std::size_t msg_len)
    : msg_len_{msg_len}, state_{}, sock_{}, addr_{.sin_family = AF_INET,
                                                  .sin_port = ::htons(port)},
      input_thr_(input_thread, this), output_thr_(output_thread, this) {
	struct addrinfo *it, *res,
	    hints{.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};

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

state Conn::status() { return state_; }

void Conn::plug() {
	// assert(state_ == plugable);
	sock_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock_ < 0) {
		std::cerr << "socket() failed" << std::endl;
		return;
	}

	state_ = connectable;
}

void Conn::unplug() {
	state_ = plugable;
	if (::close(sock_) != 0) {
		std::cerr << "close() failed: " << errno << std::endl;
	}
}

void Conn::connect() {
	// assert(state_ == connectable);
	const auto *addr = reinterpret_cast<const sockaddr *>(&addr_);
	const auto ret = ::connect(sock_, addr, sizeof *addr);
	if (ret < 0) {
		if (errno != ECONNREFUSED) {
			std::cerr << "connect() failed: " << errno << std::endl;
		}
		return;
	}

	state_ = connected;
	connect_wait_.notify_all();
	std::this_thread::yield();
}

void Conn::disconnect() {
	// if (state_ == disconnectable) {
	state_ = unplugable;
	if (::shutdown(sock_, SHUT_RDWR) != 0) {
		std::cerr << "shutdown() failed: " << errno << std::endl;
	}
	//}
}
void Conn::send(std::string &&msg) { output_chn_.send(std::move(msg)); }

std::optional<std::pair<std::unique_ptr<char[]>, std::size_t>> Conn::recv() {
	return input_chn_.recv();
}

} // namespace conn
