#ifndef CONN_CONN_HPP
#define CONN_CONN_HPP

#include "Chn.hpp"
#include "state/T.hpp"

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

class Conn { // NOLINT

	const std::size_t msg_len_;

	std::atomic<state::T> state_;

	int sock_;
	struct sockaddr_in addr_;

	Chn<std::pair<std::unique_ptr<char[]>, std::size_t>> input_chn_;
	Chn<std::string> output_chn_;
	std::thread input_thr_, output_thr_;

	std::mutex connect_mtx_;
	std::condition_variable connect_wait_;

	static void input_thread(Conn *conn);
	static void output_thread(Conn *conn);

public:
	Conn(const std::string &host, std::uint16_t port,
	     std::size_t msg_len = 65536);

	~Conn();

	void terminate();

	state::T status();

	void plug();
	void unplug();

	void connect();
	void disconnect();

	void send(std::string &&msg);

	std::optional<std::pair<std::unique_ptr<char[]>, std::size_t>> recv();
};

} // namespace conn

#endif /* CONN_CONN_HPP */
