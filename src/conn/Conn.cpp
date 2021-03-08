#include "conn/Conn.hpp"
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

#include <spdlog/spdlog.h>

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

static inline bool ended(const state::T s) {
	return s == state::terminated || s == state::unknown;
}

static inline bool stable(const state::T s) {
	return s == state::connected || ended(s);
}

void Conn::input_thread(Conn *conn) {
	for (;;) {
		if (!stable(conn->state_)) {
			std::unique_lock<std::mutex> u_lock(conn->connect_mtx_);
			// Next line blocks
			conn->connect_wait_.wait(u_lock,
			                         [conn]() { return stable(conn->state_); });

			if (ended(conn->state_)) {
				break;
			}
		}

		auto *mem = new char[conn->msg_len_];

		// Next line blocks
		const auto rc = ::recv(conn->sock_, mem, conn->msg_len_, 0);

		if (ended(conn->state_)) {
			delete[] mem;
			break;
		}

		if (rc == 0) {
			conn->state_ = state::unplugable;
			delete[] mem;
			continue;
		}

		if (rc < 0) {
			delete[] mem;

			if (errno == ECONNRESET) {
				conn->state_ = state::unplugable;
				continue;
			}

			conn->state_ = state::unknown;
			break;
		}

		auto msg = make_pair(std::unique_ptr<char[]>(mem), rc);

		conn->input_chn_.send(std::move(msg));
	}
}

void Conn::output_thread(Conn *conn) {
	for (;;) {
		if (!stable(conn->state_)) {
			std::unique_lock<std::mutex> u_lock(conn->connect_mtx_);
			// Next line blocks
			conn->connect_wait_.wait(u_lock,
			                         [conn]() { return stable(conn->state_); });

			if (ended(conn->state_)) {
				break;
			}
		}

		// Next line blocks
		const auto msg = conn->output_chn_.recv(true);

		if (ended(conn->state_)) {
			break;
		}

		if (!msg) {
			conn->state_ = state::unknown;
			break;
		}

		spdlog::debug("{}: Output\n", static_cast<void *>(conn));
		if (::send(conn->sock_, msg->data(), msg->size(), 0) < 0) {
			if (errno == EPIPE || errno == ECONNRESET) {
				conn->state_ = state::unplugable;
				continue;
			}

			conn->state_ = state::unknown;
			break;
		}
	}
}

Conn::Conn(const std::string &host, std::uint16_t port, std::size_t msg_len)
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

Conn::~Conn() { terminate(); }

void Conn::terminate() {
	if (state_ == state::terminated) {
		return;
	}

	if (state_ == state::connected || state_ == state::disconnectable) {
		disconnect();
	}

	if (state_ == state::unplugable) {
		unplug();
	}

	state_ = state::terminated;
	input_chn_.drain();
	output_chn_.drain();
	connect_wait_.notify_all();
	std::this_thread::yield();
}

state::T Conn::status() { return state_; }

void Conn::plug() {
	if (state_ == state::connectable) {
		return;
	}

	if (state_ != state::plugable) {
		std::cerr << "plug(): state is not plugable" << std::endl;
		return;
	}

	sock_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (sock_ < 0) {
		std::cerr << "socket() failed: " << errno << std::endl;
		return;
	}

	state_ = state::connectable;
}

void Conn::unplug() {
	if (state_ == state::plugable) {
		return;
	}

	if (state_ != state::unplugable) {
		std::cerr << "plug(): state is not plugable" << std::endl;
		return;
	}

	if (::close(sock_) != 0) {
		std::cerr << "close() failed: " << errno << std::endl;
		return;
	}

	state_ = state::plugable;
}

void Conn::connect() {
	if (state_ == state::connected) {
		return;
	}

	if (state_ != state::connectable) {
		std::cerr << "connect(): state is not connectable" << std::endl;
		return;
	}

	const auto *addr = reinterpret_cast<const sockaddr *>(&addr_);
	const auto ret = ::connect(sock_, addr, sizeof *addr);
	if (ret < 0) {
		if (errno != ECONNREFUSED) {
			std::cerr << "connect() failed: " << errno << std::endl;
		}
		return;
	}

	spdlog::info("{}: Connected\n", static_cast<void *>(this));

	state_ = state::connected;
	connect_wait_.notify_all();
	std::this_thread::yield();
}

void Conn::disconnect() {
	if (state_ == state::unplugable) {
		return;
	}

	if (state_ != state::connected && state_ != state::disconnectable) {
		std::cerr
		    << "disconnect(): state is neither connected nor disconnectable: "
		    << std::endl;
		return;
	}

	if (::shutdown(sock_, SHUT_RDWR) != 0) {
		std::cerr << "shutdown() failed: " << errno << std::endl;
		return;
	}

	state_ = state::unplugable;
}

void Conn::send(std::string &&msg) {
	if (state_ == state::connected) {
		spdlog::debug("{}: Send\n", static_cast<void *>(this));
		output_chn_.send(std::move(msg));
	}
}

std::optional<std::pair<std::unique_ptr<char[]>, std::size_t>> Conn::recv() {
	return input_chn_.recv();
}

} // namespace conn
