#ifndef PRAY_MATH_H
#define PRAY_MATH_H
#pragma once

#include <cmath>
#include <ostream>

struct approx
{
	approx(float x) : x(x) {}
	float x;
};
inline constexpr bool operator==(float a, const approx &approx) { return a > approx.x - 0.00001f && a < approx.x + 0.00001f; }
inline constexpr bool operator!=(float a, const approx &approx) { return !(a == approx); }

struct Vector2
{
	float x, y;
};

struct Vector3
{
	float x, y, z;

	constexpr Vector3() : x(0.f), y(0.f), z(0.f) {}
	constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}

	template<class T> constexpr float &operator[](T index) { ASSERT(index < 3); return *(&x + index); }
	template<class T> constexpr const float &operator[](T index) const { ASSERT(index < 3); return *(&x + index); }

	constexpr Vector3 operator-() const { return Vector3(-x, -y, -z); }
	constexpr Vector3 operator+(const Vector3 &a) const { return Vector3(x+a.x, y+a.y, z+a.z); }
	constexpr Vector3 operator-(const Vector3 &a) const { return Vector3(x-a.x, y-a.y, z-a.z); }
	constexpr Vector3 operator*(float a) const { return Vector3(x*a, y*a, z*a); }
	constexpr Vector3 operator/(float a) const { return Vector3(x/a, y/a, z/a); }

	constexpr Vector3 &operator*=(float a) { return *this = *this * a; }
	constexpr Vector3 &operator/=(float a) { return *this = *this / a; }

	constexpr float dot(const Vector3 &a) const { return x*a.x+y*a.y+z*a.z; }
	constexpr Vector3 cross(const Vector3 &a) const { return Vector3(y*a.z-z*a.y, z*a.x-x*a.z, x*a.y-y*a.x); }

	float lengthSquared() const { return x*x + y*y + z*z; }
	float length() const { return sqrt(lengthSquared()); }
	Vector3 &normalize() { ASSERT(length() != approx(0)); return *this /= length(); }
};

inline std::ostream &operator<<(std::ostream &o, const Vector3 &v)
{
	o << v.x << " " << v.y << " " << v.z;
	return o;
}

struct Color
{
	float r, g, b;

	constexpr Color() : r(0.f), g(0.f), b(0.f){}
	constexpr Color(float r, float g, float b) : r(r), g(g), b(b) {}

	constexpr Color operator+(const Color &a) const { return Color(r+a.r, g+a.g, b+a.b); }
	constexpr Color operator*(const Color &a) const { return Color(r*a.r, g*a.g, b*a.b); }
	constexpr Color operator*(float a) const { return Color(r*a, g*a, b*a); }

	constexpr Color &operator+=(const Color &a) { return *this = *this + a; }
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

#endif
