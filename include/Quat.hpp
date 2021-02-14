#ifndef QUAT_HPP
#define QUAT_HPP

#include "Vec3.hpp"
#include <array>
#include <cmath>

class Quat {
public:
	static constexpr Vec4 quat(const Vec4 &a) {
		const auto w2 = a.w() / 2;
		return {std::cos(w2), Vec3::mul(a, Vec4(0, std::sin(w2)))};
	}

	static constexpr void quat(Vec4 *a) {
		const auto w2 = a->w() / 2;
		a->v_[0] = std::cos(w2);
		Vec3::mul(a, Vec4(0, std::sin(w2)));
	}

	static constexpr Vec4 mul(const Vec4 &a, const Vec4 &b) {
		return {a.w() * b.w() - Vec3::dot(a, b),
		        Vec3::add(Vec3::add(Vec3::mul(b, Vec4(0, a.w())),
		                            Vec3::mul(a, Vec4(0, b.w()))),

		                  Vec3::cross(a, b))};
	}

	static constexpr void mul(Vec4 *a, const Vec4 &b) {
		auto v1 = Vec3::mul(b, Vec4(0, a->w()));
		auto v2 = Vec3::cross(*a, b);

		a->v_[0] = a->w() * b.w() - Vec3::dot(*a, b);

		Vec3::mul(a, Vec4(0, b.w()));
		Vec3::add(a, v1);
		Vec3::add(a, v2);
	}

	static constexpr Vec4 normalize(const Vec4 &a) {
		const auto disc =
		    static_cast<std::float_t>(std::pow(a.w(), 2) + Vec3::dot(a, a));
		const auto div = static_cast<std::float_t>(std::sqrt(disc));
		return {a.w() / div, Vec3::div(a, Vec4(0, div))};
	}

	static constexpr void normalize(Vec4 *a) {
		const auto disc =
		    static_cast<std::float_t>(std::pow(a->w(), 2) + Vec3::dot(*a, *a));
		const auto div = static_cast<std::float_t>(std::sqrt(disc));
		a->v_[0] /= div;
		Vec3::div(a, Vec4(0, div));
	}

	static constexpr Vec4 conj(const Vec4 &a) { return {a.w(), Vec3::neg(a)}; }

	static constexpr void conj(Vec4 *a) { Vec3::neg(a); }

	static constexpr Vec4 rotate(const Vec4 &a, const Vec4 &b) {
		return mul(mul(b, a), conj(b));
	}
};

#endif /* QUAT_HPP */
