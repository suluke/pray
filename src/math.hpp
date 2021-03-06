#ifndef PRAY_MATH_H
#define PRAY_MATH_H
#pragma once

#include <cmath>
#include <algorithm>
#include <ostream>
#include "cuda.hpp"
#include "debug.hpp"


struct approx
{
	constexpr approx(float x) : x(x) {}
	float x;
};
inline constexpr __cuda__ bool operator==(float a, const approx &approx) { return a > approx.x - 0.00001f && a < approx.x + 0.00001f; }
inline constexpr __cuda__ bool operator!=(float a, const approx &approx) { return !(a == approx); }

struct Vector3
{
	using component_t = float;
  
	component_t x, y, z;

	__cuda__ Vector3() = default;
	__cuda__ constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

	template<class T> __cuda__ component_t &operator[](T index) { ASSERT(index < 3); return *(&x + index); }
	template<class T> __cuda__ const component_t &operator[](T index) const { ASSERT(index < 3); return *(&x + index); }

	__cuda__ constexpr Vector3 operator-() const { return Vector3(-x, -y, -z); }
	__cuda__ constexpr Vector3 operator+(const Vector3 &a) const { return Vector3(x+a.x, y+a.y, z+a.z); }
	__cuda__ constexpr Vector3 operator-(const Vector3 &a) const { return Vector3(x-a.x, y-a.y, z-a.z); }
	__cuda__ constexpr Vector3 operator*(component_t a) const { return Vector3(x*a, y*a, z*a); }
	__cuda__ constexpr Vector3 operator/(component_t a) const { return Vector3(x/a, y/a, z/a); }

	__cuda__ Vector3 &operator*=(component_t a) { return *this = *this * a; }
	__cuda__ Vector3 &operator/=(component_t a) { return *this = *this / a; }

	__cuda__ constexpr component_t dot(const Vector3 &a) const { return x*a.x+y*a.y+z*a.z; }
	__cuda__ constexpr Vector3 cross(const Vector3 &a) const { return Vector3(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x); }

	__cuda__ constexpr component_t lengthSquared() const { return x*x + y*y + z*z; }
	__cuda__ component_t length() const { return sqrt(lengthSquared()); }
	__cuda__ Vector3 &normalize() { ASSERT(length() != approx(0)); return *this /= length(); }

	__cuda__ Vector3 sign() const { return Vector3(copysign(1.f, x), copysign(1.f, y), copysign(1.f, z)); }
};

static_assert(std::is_trivial<Vector3>::value, "math types should be trivial");

inline std::ostream &operator<<(std::ostream &o, const Vector3 &v)
{
	o << v.x << " " << v.y << " " << v.z;
	return o;
}

struct AABox3
{
	Vector3 min, max;

	AABox3() = default;
	constexpr AABox3(Vector3 min, Vector3 max) : min(min), max(max) {}

	template<class It>
	AABox3(const It &begin, const It &end)
	{
		clear();
		for(auto p = begin; p != end; ++p) insert(*p);
	}

	AABox3 &clear()
	{
		min = Vector3(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
		max = Vector3(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
		return *this;
	}

	AABox3 &insert(const Vector3 &other)
	{
		for(auto i=0; i<3; ++i) min[i] = std::min(min[i], other[i]);
		for(auto i=0; i<3; ++i) max[i] = std::max(max[i], other[i]);
		return *this;
	}

	AABox3 &insert(const AABox3 &other)
	{
		insert(other.min);
		insert(other.max);
		return *this;
	}

	Vector3 calculateCenter() const
	{
		return (min + max) / 2.f;
	}

	float calculateSurfaceArea() const
	{
		const auto d = max - min;
		return 2*d.x*d.y + 2*d.x*d.z + 2*d.y*d.z;
	}

	bool isValid() const
	{
		return min.x <= max.x && min.y <= max.y && min.z <= max.z;
	}

	// not using float delta here, not necessary!
	bool isZero() const
	{
		return min.x == max.x && min.y == max.y && min.z == max.z;
	}
};

static_assert(std::is_trivial<AABox3>::value, "math types should be trivial");

struct IntDimension2
{
	using dim_t = uint32_t;
	dim_t w, h;

	IntDimension2() = default;
	constexpr IntDimension2(dim_t w, dim_t h) : w(w), h(h) {}
};

static_assert(std::is_trivial<IntDimension2>::value, "math types should be trivial");

template <unsigned W, unsigned H>
struct ConstDim2 {
	static const unsigned w = W;
	static const unsigned h = H;
};

struct Color
{
	float r, g, b;

	__cuda__ Color() = default;
	__cuda__ constexpr Color(float r, float g, float b) : r(r), g(g), b(b) {}

	__cuda__ constexpr Color operator+(const Color &a) const { return Color(r+a.r, g+a.g, b+a.b); }
	__cuda__ constexpr Color operator-(const Color &a) const { return Color(r-a.r, g-a.g, b-a.b); }
	__cuda__ constexpr Color operator*(const Color &a) const { return Color(r*a.r, g*a.g, b*a.b); }
	__cuda__ constexpr Color operator*(float a) const { return Color(r*a, g*a, b*a); }
	__cuda__ constexpr Color operator/(float a) const { return Color(r/a, g/a, b/a); }

	__cuda__ Color abs(const Color &a) const { return Color(std::abs(a.r),std::abs(a.g),std::abs(a.b)); }

	__cuda__ Color &operator*=(const Color &a) { return *this = *this * a; }
	__cuda__ Color &operator+=(const Color &a) { return *this = *this + a; }
	__cuda__ Color &operator-=(const Color &a) { return *this = *this - a; }
	__cuda__ Color &operator/=(float a) { return *this = *this / a; }
};

static_assert(std::is_trivial<Color>::value, "math types should be trivial");

inline int fast_round (float f) {
	return (int)(f + 0.5f);
}
#endif
