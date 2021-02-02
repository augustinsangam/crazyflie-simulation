#ifndef CONN_HPP
#define CONN_HPP

#include <cstddef>
#include <cstdint>
#include <future>
#include <string>
#include <vector>

extern "C" {
/* struct sockaddr_in */
#include <netinet/in.h>
}

enum conn : int_fast8_t {
	ok = 0,
	unconnected = -1,
	unknown = -127,
};

class Conn { // NOLINT
	static constexpr const std::size_t buf_len{65536};

	bool connected_{};
	int sock_;
	struct sockaddr_in addr_;

public:
	Conn(const std::string &host, uint16_t port);

	~Conn();

	std::future<conn> connect();

	void disconnect();

	std::future<conn> send(const std::string &s) const;

	std::future<conn> send(const std::pair<const char *, size_t> &buf) const;

	std::future<conn> send(std::unique_ptr<std::vector<char>> v) const;

	std::future<std::pair<char *, ssize_t>> recv() const;
};

#endif /* CONN_HPP */
