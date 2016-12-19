#ifndef PRAY_MATH_H
#define PRAY_MATH_H
#pragma once

#include <cmath>
#include <algorithm>
#include <ostream>
#include "debug.hpp"


struct approx
{
	constexpr approx(float x) : x(x) {}
	float x;
};
inline constexpr bool operator==(float a, const approx &approx) { return a > approx.x - 0.00001f && a < approx.x + 0.00001f; }
inline constexpr bool operator!=(float a, const approx &approx) { return !(a == approx); }

struct Vector3
{
	using component_t = float;
  
	component_t x, y, z;

	Vector3() {}
	constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

	template<class T> component_t &operator[](T index) { ASSERT(index < 3); return *(&x + index); }
	template<class T> constexpr const component_t &operator[](T index) const { ASSERT(index < 3); return *(&x + index); }

	constexpr Vector3 operator-() const { return Vector3(-x, -y, -z); }
	constexpr Vector3 operator+(const Vector3 &a) const { return Vector3(x+a.x, y+a.y, z+a.z); }
	constexpr Vector3 operator-(const Vector3 &a) const { return Vector3(x-a.x, y-a.y, z-a.z); }
	constexpr Vector3 operator*(component_t a) const { return Vector3(x*a, y*a, z*a); }
	constexpr Vector3 operator/(component_t a) const { return Vector3(x/a, y/a, z/a); }

	Vector3 &operator*=(component_t a) { return *this = *this * a; }
	Vector3 &operator/=(component_t a) { return *this = *this / a; }

	constexpr component_t dot(const Vector3 &a) const { return x*a.x+y*a.y+z*a.z; }
	constexpr Vector3 cross(const Vector3 &a) const { return Vector3(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x); }

	constexpr component_t lengthSquared() const { return x*x + y*y + z*z; }
	component_t length() const { return sqrt(lengthSquared()); }
	Vector3 &normalize() { ASSERT(length() != approx(0)); return *this /= length(); }
};

inline std::ostream &operator<<(std::ostream &o, const Vector3 &v)
{
	o << v.x << " " << v.y << " " << v.z;
	return o;
}

struct AABox3
{
	Vector3 min, max;

	AABox3() {}
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

struct IntDimension2
{
	using dim_t = uint32_t;
	dim_t w, h;

	IntDimension2() {}
	constexpr IntDimension2(dim_t w, dim_t h) : w(w), h(h) {}
};

template <unsigned W, unsigned H>
struct ConstDim2 {
	static const unsigned w = W;
	static const unsigned h = H;
};

struct Color
{
	float r, g, b;

	Color() {}
	constexpr Color(float r, float g, float b) : r(r), g(g), b(b) {}

	constexpr Color operator+(const Color &a) const { return Color(r+a.r, g+a.g, b+a.b); }
	constexpr Color operator-(const Color &a) const { return Color(r-a.r, g-a.g, b-a.b); }
	constexpr Color operator*(const Color &a) const { return Color(r*a.r, g*a.g, b*a.b); }
	constexpr Color operator*(float a) const { return Color(r*a, g*a, b*a); }
	constexpr Color operator/(float a) const { return Color(r/a, g/a, b/a); }

	Color &operator+=(const Color &a) { return *this = *this + a; }
	Color &operator-=(const Color &a) { return *this = *this - a; }
};

inline int fast_round (float f) {
	return (int)(f + 0.5f);
}
#endif
