#ifndef PRAY_RAY_HPP
#define PRAY_RAY_HPP
#pragma once
#include "scene.hpp" // includes math and image
#include "sse_math.hpp"

template<class scene_t>
struct Ray {
  using dim_t = IntDimension2::dim_t;
  using intersect_t = TriangleIndex;
  using color_t = Color;
  using location_t = Vector3;
  using vec3_t = Vector3;
  using distance_t = float;
  using angle_t = float;
  using bool_t = bool;
  using material_t = MaterialIndex;

  using dim = ConstDim2<1, 1>;

  static constexpr unsigned subrays_count = 1u;

  const Vector3 origin;
  const Vector3 direction;
  const Vector3 dir_inv;

  Ray(location_t origin, Vector3 direction) : origin(origin), direction(direction), dir_inv(1.f / direction.x, 1.f / direction.y, 1.f / direction.z) {}

  inline bool_t intersectTriangle(const Triangle &triangle, distance_t *out_distance) const
  {
    // http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
    const __m128 origin_mm = set_vector3_ps(origin);
    const __m128 direction_mm = set_vector3_ps(direction);

    const __m128 t_v1 = set_vector3_ps(triangle.vertices[0]);
    const __m128 t_v2 = set_vector3_ps(triangle.vertices[1]);
    const __m128 t_v3 = set_vector3_ps(triangle.vertices[2]);

    const __m128 e1 = _mm_sub_ps(t_v2, t_v1);
    const __m128 e2 = _mm_sub_ps(t_v3, t_v1);

    const __m128 p = cross_ps(direction_mm, e2);

    const float det = fast_dot_ps(p, e1);

    if(det == approx(0.f)) return false;

    const __m128 t = _mm_sub_ps(origin_mm, t_v1);

    const __m128 q = cross_ps(t, e1);

    const float inv_det = 1.f / det;

    const float u = fast_dot_ps(p, t) * inv_det;
    const float v = fast_dot_ps(q, direction_mm) * inv_det;

    if(u >= 0.f && v >= 0.f && u + v <= 1.f)
    {
      const float distance = fast_dot_ps(q, e2) * inv_det;
      *out_distance = distance;
      return distance >= 0.f;
    }

    return false;
  }
  
  inline bool_t intersectAABB(const AABox3 &aabb) const
  {
    // http://psgraphics.blogspot.de/2016/02/new-simple-ray-box-test-from-andrew.html
    float t_min = std::numeric_limits<float>::lowest(), t_max = std::numeric_limits<float>::max();
    for(int i=0; i<3; ++i)
    {
      float i_d = dir_inv[i];
      float t0 = (aabb.min[i] - origin[i]) * i_d;
      float t1 = (aabb.max[i] - origin[i]) * i_d;
      if(i_d < 0.f) std::swap(t0, t1);
      t_min = std::max(t_min, t0);
      t_max = std::min(t_max, t1);
      if(t_max < t_min) return false;
    }
    return true;
  }

  location_t getIntersectionPoint(distance_t intersection_distance) const {
    return origin + direction * intersection_distance;
  }

  Vector3 getSubrayDirection(unsigned subray) const { ASSERT(subray == 0); return direction; }

  void isDirectionSignEqualForAllSubrays(Vector3 test_sign, std::array<bool, 3> *out_result) const {
    const auto sign = direction.sign();
    (*out_result)[0] = test_sign.x == sign.x;
    (*out_result)[1] = test_sign.y == sign.y;
    (*out_result)[2] = test_sign.z == sign.z;
  }

  bool_t intersectAxisPlane(float plane, unsigned axis, distance_t maximum_distance) const {
    const auto o = origin[axis];
    const auto d_inv = dir_inv[axis];
    const auto p = plane;

    // solve o + t*d = p -> t = (p - o) / d
    const auto t = (p - o) * d_inv;

    // no t > 0 test for now, breaks if the camera is inside the scene aabb...
    return t < maximum_distance;
  }

  static Ray getShadowRay(Light light, location_t P, distance_t *ld) {
    const Vector3 light_vector = light.position - P;
		const float light_distance = light_vector.length();
		const Vector3 L = light_vector / light_distance;
    *ld = light_distance;
    return {P + L * 0.001f, L};
  }

  static inline color_t getMaterialColors(const scene_t &scene, intersect_t triangle) {
    const auto material_index = scene.triangles[triangle].material_index;
    ASSERT(material_index != MaterialIndex_Invalid);
    return scene.materials[material_index].color;
  }

  static inline vec3_t getNormals(const scene_t &scene, intersect_t triangle) {
    return scene.triangles[triangle].calculateNormal();
  }
  
  static color_t shade(const scene_t &scene, const location_t &P, intersect_t triangle, const Light &light, distance_t intersection_distance, vec3_t N, color_t mat_color) {
    // TODO duplicated code
    const Vector3 light_vector = light.position - P;
    const float light_distance = light_vector.length();
    const Vector3 L = light_vector / light_distance;

    if (intersection_distance < light_distance)
      return {0.f, 0.f, 0.f};

    return mat_color * light.color * L.dot(N) / (light_distance * light_distance);
  }

  static void addBackgroundcolor(color_t &result_color, const intersect_t intersected_triangle, const Color bg) {
    // can be no-op because the tracer already returns the background color if there have not been __ANY__ intersections
  }

  static inline distance_t max_distance() {
    return std::numeric_limits<distance_t>::max();
  }

  static inline void updateIntersections(intersect_t *intersected_triangle, TriangleIndex triangle_index, distance_t *minimum_distance, distance_t distance) {
    if(distance < *minimum_distance)
    {
      *intersected_triangle = triangle_index;
      *minimum_distance = distance;
    }
  }

  static inline bool isNoIntersection(intersect_t intersect) {
    return intersect == TriangleIndex_Invalid;
  }

  static inline intersect_t getNoIntersection() {
    return TriangleIndex_Invalid;
  }

  static inline bool isAll(bool_t b) {
    return b;
  }

  static inline bool isAny(bool_t b) {
    return b;
  }

  static inline bool_t booleanAnd(bool_t a, bool_t b) {
    return a && b;
  }

  static inline bool_t booleanNot(bool_t a) {
    return !a;
  }

  static inline bool_t getTrue() {
    return true;
  }

  /**
   * True if angle between v1 and v2 is larger than 90 degrees
   */
  static inline bool_t isOppositeDirection(const vec3_t v1, const vec3_t v2) {
    return v1.dot(v2) <= 0.f;
  }
};

template<class scene_t>
inline std::ostream &operator<<(std::ostream &o, const Ray<scene_t> &r) {
  o << "Origin: " << r.origin << "\nDirection: " << r.direction;
  return o;
}
#endif // PRAY_RAY_HPP
