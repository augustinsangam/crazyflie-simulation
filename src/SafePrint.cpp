#include "SafePrint.hpp"
#include <thread>

std::mutex SafePrint::mtx_;

SafePrint &SafePrint::endl(SafePrint &sp) {
	// sp.oss_ << sp.oss_.widen('\n');
	{
		std::lock_guard<std::mutex> lock(mtx_);
		sp.os_ << "[" << std::this_thread::get_id() << "] " << sp.oss_.str()
		       << std::endl;
	}
	sp.oss_.str("");
	return sp;
}

SafePrint::SafePrint(std::ostream &os) : os_{os} {}

SafePrint &SafePrint::operator<<(const std::string &s) {
	oss_ << s;
	return *this;
}

/*template <typename T> SafePrint &SafePrint::operator<<(const T &s) {
    oss_ << s;
    return *this;
}*/

using MyStreamManipulator = SafePrint &(*)(SafePrint &);

SafePrint &SafePrint::operator<<(MyStreamManipulator fn) { return fn(*this); }
