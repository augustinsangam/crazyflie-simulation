#include "Proxy.hpp"
#include <cstdlib> /* std::getenv */

#include <spdlog/spdlog.h>
#include <tao/json/external/pegtl/parse_error.hpp>
#include <tao/json/from_string.hpp>

static std::string get_env(const std::string &var, const std::string &val) {
	const auto *var_val = std::getenv(var.c_str());
	return var_val ? var_val : val;
}

static const std::unordered_map<std::string, cmd::T> cmd_map{
    {"", cmd::none},
    {"darken", cmd::darken},
    {"land", cmd::land},
    {"lighten", cmd::lighten},
    {"pulse", cmd::pulse},
    {"startMission", cmd::start_mission},
    {"takeOff", cmd::take_off},
};

static std::shared_ptr<conn::Conn> conn_{};

decltype(Proxy::recv_boxes_) Proxy::recv_boxes_{};

Proxy::Proxy(std::string name) : name_{std::move(name)} {
	recv_boxes_.insert(decltype(recv_boxes_)::value_type(name_, &recv_box_));
	spdlog::debug("{}", name_);

	if (conn_ == nullptr) {
		conn_ = std::make_shared<conn::Conn>(
		    get_env("HOST", "localhost"),
		    static_cast<std::uint16_t>(std::stoul(get_env("PORT", "3995"))),
		    Proxy::recv_cb);
		spdlog::debug("Connection created");
	}
}

void Proxy::send(std::string &&msg) {
	switch (conn_->status()) {
	case conn::state::init:
		spdlog::error("Connection is not ready!");
		break;
	case conn::state::plugable:
		conn_->plug();
		break;
	case conn::state::connectable:
		conn_->connect();
		break;
	case conn::state::connected:
		conn_->send(std::move(msg));
		break;
	case conn::state::disconnectable:
		conn_->disconnect();
		break;
	case conn::state::unplugable:
		conn_->unplug();
		break;
	case conn::state::terminated:
		spdlog::warn("connection terminated");
		break;
	case conn::state::unknown:
		spdlog::error("connection in an unknown state");
		break;
	}
}

void Proxy::recv_cb(gen_buf_t &&msg) {
	spdlog::trace("recv_cb entered");

	try {

		const auto v = tao::json::from_string(msg.first.get(), msg.second);

		const auto &m = v.get_object();

		const auto &name = m.at("data").at("name").get_string();

		const auto it = recv_boxes_.find(name);
		if (it == recv_boxes_.end()) {
			return;
		}

		const auto &type = m.at("type").get_string();

		const auto type_it = cmd_map.find(type);
		auto cmd = type_it != cmd_map.end() ? type_it->second : cmd::unknown;
		it->second->push(cmd);
	} catch (const tao::json::pegtl::parse_error &e) {
		spdlog::warn("Invalid json received ({}) : {}", e.what(),
		             msg.first.get());
	}
}
