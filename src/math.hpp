#ifndef PRAY_MATH_H
#define PRAY_MATH_H
#pragma once

#include <cmath>
#include <algorithm>
#include <ostream>

struct approx
{
	constexpr approx(float x) : x(x) {}
	float x;
};
inline constexpr bool operator==(float a, const approx &approx) { return a > approx.x - 0.00001f && a < approx.x + 0.00001f; }
inline constexpr bool operator!=(float a, const approx &approx) { return !(a == approx); }

struct Vector3
{
	float x, y, z;

	Vector3() {}
	constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

	template<class T> float &operator[](T index) { ASSERT(index < 3); return *(&x + index); }
	template<class T> constexpr const float &operator[](T index) const { ASSERT(index < 3); return *(&x + index); }

	constexpr Vector3 operator-() const { return Vector3(-x, -y, -z); }
	constexpr Vector3 operator+(const Vector3 &a) const { return Vector3(x+a.x, y+a.y, z+a.z); }
	constexpr Vector3 operator-(const Vector3 &a) const { return Vector3(x-a.x, y-a.y, z-a.z); }
	constexpr Vector3 operator*(float a) const { return Vector3(x*a, y*a, z*a); }
	constexpr Vector3 operator/(float a) const { return Vector3(x/a, y/a, z/a); }

	Vector3 &operator*=(float a) { return *this = *this * a; }
	Vector3 &operator/=(float a) { return *this = *this / a; }

	constexpr float dot(const Vector3 &a) const { return x*a.x+y*a.y+z*a.z; }
	constexpr Vector3 cross(const Vector3 &a) const { return Vector3(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x); }

	constexpr float lengthSquared() const { return x*x + y*y + z*z; }
	float length() const { return sqrt(lengthSquared()); }
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

struct Color
{
	float r, g, b;

	Color() {}
	constexpr Color(float r, float g, float b) : r(r), g(g), b(b) {}

	constexpr Color operator+(const Color &a) const { return Color(r+a.r, g+a.g, b+a.b); }
	constexpr Color operator*(const Color &a) const { return Color(r*a.r, g*a.g, b*a.b); }
	constexpr Color operator*(float a) const { return Color(r*a, g*a, b*a); }

	Color &operator+=(const Color &a) { return *this = *this + a; }
};

struct IntDimension2
{
	using dim_t = uint32_t;
	dim_t w, h;

	IntDimension2() {}
	IntDimension2(dim_t w, dim_t h) : w(w), h(h) {}
};

inline bool intersectRayTriangle(const Vector3 &r_o, const Vector3 &r_d, const Vector3 &t_v1, const Vector3 &t_v2, const Vector3 &t_v3, float *out_distance)
{
	// http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf

	const Vector3 e1 = t_v2 - t_v1;
	const Vector3 e2 = t_v3 - t_v1;

	const Vector3 p = r_d.cross(e2);

	const float det = p.dot(e1);

	if(det == approx(0.f)) return false;

	const Vector3 t = r_o - t_v1;

	const Vector3 q = t.cross(e1);

	const float u = p.dot(t) / det;
	const float v = q.dot(r_d) / det;

	if(u >= 0.f && v >= 0.f && u + v <= 1.f)
	{
		const float distance = q.dot(e2) / det;
		*out_distance = distance;
		return distance >= 0.f;
	}

	return false;
}

inline bool intersectRayAABB(const Vector3 &r_o, const Vector3 &r_d, const AABox3 &aabb)
{
	// http://psgraphics.blogspot.de/2016/02/new-simple-ray-box-test-from-andrew.html

	float t_min = std::numeric_limits<float>::lowest(), t_max = std::numeric_limits<float>::max();

	for(int i=0; i<3; ++i)
	{
		float i_d = 1.f / r_d[i];
		float t0 = (aabb.min[i] - r_o[i]) * i_d;
		float t1 = (aabb.max[i] - r_o[i]) * i_d;
		if(i_d < 0.f) std::swap(t0, t1);
		t_min = std::max(t_min, t0);
		t_max = std::min(t_max, t1);
		if(t_max < t_min) return false;
	}

	return true;
}

#endif
