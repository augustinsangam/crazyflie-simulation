#ifndef PROXY_HPP
#define PROXY_HPP

#include "SharedQueue.hpp"
#include "cmd/T.hpp"
#include "conn/Conn.hpp"
#include "gen_buf.hpp"
#include <memory> /* std::shared_ptr */
#include <optional>
#include <string>
#include <unordered_map>

class Proxy {
	SharedQueue<cmd::T> recv_box_;
	std::string name_;

	static std::unordered_map<std::string, decltype(recv_box_) *> recv_boxes_;

public:
	explicit Proxy(std::string name);
	inline auto next_cmd() { return recv_box_.pop(); }
	void send(std::string &&msg); // TODO: Remove RTStatus

	static void recv_cb(gen_buf_t &&msg);
};

#endif /* PROXY_HPP */
