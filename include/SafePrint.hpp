#ifndef SAFEPRINT_HPP
#define SAFEPRINT_HPP

#include <ios>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

class SafePrint {
	static std::mutex mtx_;

	std::ostream &os_;
	std::ostringstream oss_;

public:
	static SafePrint &endl(SafePrint &sp);

	explicit SafePrint(std::ostream &os);

	SafePrint &operator<<(const std::string &s);

	// template <typename T> SafePrint &operator<<(const T &s);

	using MyStreamManipulator = SafePrint &(*)(SafePrint &);

	SafePrint &operator<<(MyStreamManipulator fn);
};

#endif /* SAFEPRINT_HPP */
