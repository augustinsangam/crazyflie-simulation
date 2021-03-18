#ifndef SHAREDQUEUE_HPP
#define SHAREDQUEUE_HPP

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <queue>

template <typename T> class SharedQueue {
	std::mutex m_;
	std::queue<T> q_;

public:
	void push(T t) {
		std::lock_guard<std::mutex> u_lock(m_);
		q_.push(t);
	}

	void push(T &&t) {
		std::lock_guard<std::mutex> u_lock(m_);
		q_.push(std::move(t));
	}

	std::uint_fast32_t size() {
		std::lock_guard<std::mutex> u_lock(m_);
		return q_.size();
	}

	std::optional<T> pop() {
		std::lock_guard<std::mutex> u_lock(m_);
		if (q_.empty()) {
			return std::nullopt;
		}

		auto val = std::move(q_.front());
		q_.pop();
		return val;
	}
};

#endif /* SHAREDQUEUE_HPP */
