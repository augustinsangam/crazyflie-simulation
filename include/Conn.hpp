#ifndef CONN_HPP
#define CONN_HPP

#include "Chn.hpp"

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <future>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

extern "C" {
/* struct sockaddr_in */
#include <netinet/in.h>
}

namespace conn {

enum state : int_fast8_t {
	ok = 0,
	unconnected = -1,
	unknown = -127,
};

class Conn { // NOLINT
	bool connected_{};
	std::size_t msg_len_;

	int sock_;
	struct sockaddr_in addr_;

	Chn<std::pair<std::unique_ptr<char[]>, std::size_t>> input_chn_;
	Chn<std::string> output_chn_;
	std::thread input_thr_, output_thr_;

	std::mutex connexion_mtx_;
	std::condition_variable connexion_wait_;

	static void input_thread(Conn *conn);
	static void output_thread(Conn *conn);

public:
	Conn(const std::string &host, uint16_t port, std::size_t msg_len = 65536);

	~Conn();

	state connect();

	void disconnect();

	void send(std::string &&msg);

	std::optional<std::pair<std::unique_ptr<char[]>, std::size_t>> recv();
};

} // namespace conn

#endif /* CONN_HPP */
