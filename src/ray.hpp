#include "scene.hpp" // includes math and image

struct Ray {
  using dim_t = IntDimension2::dim_t;
  using intersect_t = TriangleIndex;
  using color_t = Color;
  using location_t = Vector3;
  using vec3_t = Vector3;
  using distance_t = float;
  using angle_t = float;
  using bool_t = bool;
  
  static constexpr IntDimension2 dim = {1, 1};
  
  const Vector3 origin;
  const Vector3 direction;

  Ray(const Camera &cam, const Vector3 &left, const Vector3 &top, const dim_t x, const dim_t y, float max_x, float max_y)
  : origin(cam.position), direction((top * (1.f - (2 * y + 1) / max_y) + left * (1.f - (2 * x + 1) / max_x) + cam.direction).normalize()) {}

  inline bool_t intersectTriangle(const Triangle &triangle, distance_t *out_distance) const
  {
    // http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
    const Vector3 &t_v1 = triangle.vertices[0];
    const Vector3 &t_v2 = triangle.vertices[1];
    const Vector3 &t_v3 = triangle.vertices[2];

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
  
  inline bool_t intersectAABB(const AABox3 &aabb) const
  {
    // http://psgraphics.blogspot.de/2016/02/new-simple-ray-box-test-from-andrew.html
    float t_min = std::numeric_limits<float>::lowest(), t_max = std::numeric_limits<float>::max();
    for(int i=0; i<3; ++i)
    {
      float i_d = 1.f / direction[i];
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

#ifndef DEBUG // for debugging tool...
private:
#endif
  Ray(location_t origin, Vector3 direction) : origin(origin), direction(direction) {}

public:
  static Ray getShadowRay(Light light, location_t P, distance_t *ld) {
    const Vector3 light_vector = light.position - P;
		const float light_distance = light_vector.length();
		const Vector3 L = light_vector / light_distance;
    *ld = light_distance;
    return {P + L * 0.001f, L};
  }

  static vec3_t getNormals(const Scene &scene, intersect_t triangle) {
    return scene.triangles[triangle].calculateNormal();
  }
  
  static color_t shade(const Scene &scene, const location_t &P, intersect_t triangle, const Light &light, distance_t intersection_distance, vec3_t N) {
    // TODO duplicated code
    const Vector3 light_vector = light.position - P;
		const float light_distance = light_vector.length();
		const Vector3 L = light_vector / light_distance;

    if (intersection_distance < light_distance)
      return {0.f, 0.f, 0.f};

    const auto material_index = scene.triangles[triangle].material_index;
    ASSERT(material_index != MaterialIndex_Invalid);
    
    return scene.materials[material_index].color * light.color * (std::max(L.dot(N), 0.f) / (light_distance * light_distance));
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

  /**
   * True if angle between v1 and v2 is larger than 90 degrees
   */
  static inline bool_t isOppositeDirection(const vec3_t v1, const vec3_t v2) {
    return v1.dot(v2) <= 0.f;
  }
};

inline std::ostream &operator<<(std::ostream &o, const Ray &r) {
  o << "Origin: " << r.origin << "\nDirection: " << r.direction;
  return o;
}
