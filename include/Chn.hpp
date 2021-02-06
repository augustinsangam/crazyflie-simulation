#ifndef CHN_HPP
#define CHN_HPP

#include <condition_variable>
#include <future>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>

/* https://github.com/Balnian/ChannelsCPP/blob/master/ChannelsCPP/ChannelBuffer.h
 */

class channel_closed : public std::runtime_error {
	using std::runtime_error::runtime_error;
};

template <typename T> class Chn {
private:
	std::atomic_bool is_open_{true};
	std::condition_variable input_wait_, output_wait_;
	std::queue<T> q_;
	std::mutex m_;

public:
	~Chn() { drain(); }

	void drain() {
		is_open_.store(false);
		input_wait_.notify_one();
		output_wait_.notify_all();
	}

	bool is_open() { return is_open_ || !q_.empty(); }

	Chn &operator<<(T val) {
		if (!is_open_) {
			throw channel_closed("send attempt while closed");
		}

		{
			std::lock_guard<std::mutex> u_lock(m_);
			// TODO: is queue full
			q_.push(std::move(val));
		}
		input_wait_.notify_one();
		return *this;
	}

	Chn &operator>>(std::optional<T> &val) {
		std::unique_lock<std::mutex> u_lock(m_);
		if (q_.empty()) {
			if (!is_open_) {
				val = std::nullopt;
				return *this;
			}

			input_wait_.wait(u_lock,
			                 [&]() { return !q_.empty() || !is_open_; });

			if (q_.empty() && !is_open_) {
				val = std::nullopt;
				return *this;
			}
		}

		val = std::move(q_.front());
		q_.pop();
		output_wait_.notify_one();
		return *this;
	}
};

#endif /* CHN_HPP */
