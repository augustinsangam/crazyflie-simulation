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

void Conn::sender_thread(Conn *conn) {
	std::cout << "sender_thread: created" << std::endl;
	conn->sender_wait();
	std::cout << "sender_thread: connected" << std::endl;

	for (;;) {
		conn->sender_wait();
		const auto wc = ::send(conn->sock_, conn->next_msg_.first,
		                       conn->next_msg_.second, 0);
		delete[] conn->next_msg_.first; // NOLINT
		conn->next_msg_.first = nullptr;
		if (wc < 0) {
			std::cerr << "sender_thread: send failed with " << wc << std::endl;
			break;
		}
	}
}

void Conn::receiver_thread(Conn *conn) {
	std::cout << "receiver_thread: created" << std::endl;
	conn->receiver_wait();
	std::cout << "receiver_thread: connected" << std::endl;

	for (;;) {
		auto *buf = new char[buf_len]; // NOLINT
		const auto rc = ::recv(conn->sock_, buf, buf_len, 0);
		if (rc < 0) {
			std::cerr << "receiver_thread: recv failed with " << rc
			          << std::endl;
			delete[] buf; // NOLINT
			break;
		}

		conn->c_->call(std::make_pair(buf, static_cast<std::size_t>(rc)));
	}

	std::cout << "receiver_thread: done" << std::endl;
}

Conn::Conn(const std::string &host, uint16_t port, Callable *c)
    : sock_{::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP)},
      addr_{.sin_family = AF_INET, .sin_port = ::htons(port)}, c_{c},
      sender_flag_{}, receiver_flag_{}, sender_thread_(sender_thread, this),
      receiver_thread_(receiver_thread, this), next_msg_(nullptr, 0) {
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
		sender_notify();
		receiver_notify();
		// FIXME:
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}
	return status >= 0 ? conn::ok : conn::unknown;
}

void Conn::disconnect() {
	if (connected_) {
		// connected_ = false;
		//::close(sock_);
	}
}

void Conn::send(msg_t msg) {
	if (next_msg_.first != nullptr) {
		std::cerr << "sending skipped" << std::endl;
		return;
	}

	next_msg_.first = new char[msg.second]; // NOLINT
	next_msg_.second = msg.second;
	std::memcpy(next_msg_.first, msg.first, msg.second);
	sender_notify();
}

void Conn::sender_wait() {
	std::unique_lock<std::mutex> lock(sender_mutex_);
	sender_cond_.wait_for(lock, std::chrono::hours(512),
	                      [&]() { return sender_flag_; });
	sender_flag_ = false;
}

void Conn::receiver_wait() {
	std::unique_lock<std::mutex> lock(receiver_mutex_);
	receiver_cond_.wait_for(lock, std::chrono::hours(512),
	                        [&]() { return receiver_flag_; });
	receiver_flag_ = false;
}

void Conn::sender_notify() {
	std::lock_guard<std::mutex> lock(sender_mutex_);
	sender_flag_ = true;
	sender_cond_.notify_one();
}

void Conn::receiver_notify() {
	std::lock_guard<std::mutex> lock(receiver_mutex_);
	receiver_flag_ = true;
	receiver_cond_.notify_one();
}

} // namespace conn
