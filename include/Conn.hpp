#ifndef CONN_HPP
#define CONN_HPP

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

using msg_t = std::pair<const char *, std::size_t>;

using mut_msg_t = std::pair<char *, std::size_t>;

class Callable {
public:
	virtual void call(mut_msg_t) = 0;
};

enum state : int_fast8_t {
	ok = 0,
	unconnected = -1,
	unknown = -127,
};

class Conn { // NOLINT
	static constexpr const std::size_t buf_len{65536};

	bool connected_{};
	int sock_;
	struct sockaddr_in addr_;
	Callable *c_;

	bool sender_flag_, receiver_flag_;
	std::thread sender_thread_, receiver_thread_;
	std::mutex sender_mutex_, receiver_mutex_;
	std::condition_variable sender_cond_, receiver_cond_;

	mut_msg_t next_msg_;

public:
	Conn(const std::string &host, uint16_t port, Callable *c);

	~Conn();

	state connect();

	void disconnect();

	void send(msg_t msg);

	void sender_wait();
	void receiver_wait();

	void sender_notify();
	void receiver_notify();

	static void sender_thread(Conn *conn);
	static void receiver_thread(Conn *conn);
};

} // namespace conn

#endif /* CONN_HPP */
