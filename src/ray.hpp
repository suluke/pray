#include "simd.hpp"

struct Ray {
	Vector3 origin;
	Vector3 direction;

	Ray(const Vector3 &_origin, const Vector3 &_direction) : origin(_origin), direction(_direction) {}

  inline bool intersectTriangle(const Vector3 &t_v1, const Vector3 &t_v2, const Vector3 &t_v3, float *out_distance) const
  {
    // http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf

    const Vector3 e1 = t_v2 - t_v1;
    const Vector3 e2 = t_v3 - t_v1;

    const Vector3 p = direction.cross(e2);

    const float det = p.dot(e1);

    if(det == approx(0.f)) return false;

    const Vector3 t = origin - t_v1;

    const Vector3 q = t.cross(e1);

    const float u = p.dot(t) / det;
    const float v = q.dot(direction) / det;

    if(u >= 0.f && v >= 0.f && u + v <= 1.f)
    {
      const float distance = q.dot(e2) / det;
      *out_distance = distance;
      return distance >= 0.f;
    }

    return false;
  }
};
