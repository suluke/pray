#include "simd.hpp"
#include <array>

struct SSEColor : public simd::Vec3Pack {
  SSEColor(const Color &c) : simd::Vec3Pack(c.r, c.g, c.b) {}
  SSEColor(const simd::Vec3Pack &c) {}
  SSEColor(simd::floatty r, simd::floatty g, simd::floatty b) : simd::Vec3Pack(r, g, b) {}
};

struct SSERay {
  using dim_t = IntDimension2::dim_t;
  using intersect_t = simd::intty;
  using color_t = SSEColor;
  using location_t = simd::Vec3Pack;
  using vec3_t = simd::Vec3Pack;
  using distance_t = simd::floatty;
  using angle_t = simd::floatty;
  using bool_t = simd::intty;
  
  static constexpr IntDimension2 dim = {simd::REGISTER_CAPACITY_FLOAT == 8 ? 4 : 2, 2};

  const location_t origin;
  simd::Vec3Pack direction;
  simd::Vec3Pack dir_inv;

  SSERay(const Camera &cam, const Vector3 &left, const Vector3 &top, const dim_t x, const dim_t y, float max_x, float max_y) : origin(cam.position) {
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> X;
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> Y;
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> Z;
    for (unsigned i = 0; i < simd::REGISTER_CAPACITY_FLOAT; ++i) {
      X[i] = cam.direction.x;
      Y[i] = cam.direction.y;
      Z[i] = cam.direction.z;
    }
    for (unsigned i_x = 0; i_x < dim.w; ++i_x) {
      float f_x = 1.f - (2 * (x + i_x) + 1) / max_x;
      auto l = left * f_x;
      for (unsigned i_y = 0; i_y < dim.h; ++i_y) {
        X[i_y * dim.w + i_x] += l.x;
        Y[i_y * dim.w + i_x] += l.y;
        Z[i_y * dim.w + i_x] += l.z;
      }
    }
    for (unsigned i_y = 0; i_y < dim.h; ++i_y) {
      float f_y = 1.f - (2 * (y + i_y) + 1) / max_y;
      auto t = top * f_y;
      for (unsigned i_x = 0; i_x < dim.w; ++i_x) {
        X[i_y * dim.w + i_x] += t.x;
        Y[i_y * dim.w + i_x] += t.y;
        Z[i_y * dim.w + i_x] += t.z;
      }
    }
    direction.x = simd::load_ps(&X[0]);
    direction.y = simd::load_ps(&Y[0]);
    direction.z = simd::load_ps(&Z[0]);
    direction.normalize();
    dir_inv = vec3_t(1.f, 1.f, 1.f) / direction;
  }

  inline bool_t intersectTriangle(const Triangle &triangle, distance_t *out_distance) const {
    // http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
    const Vector3 &t_v1 = triangle.vertices[0];
    const Vector3 &t_v2 = triangle.vertices[1];
    const Vector3 &t_v3 = triangle.vertices[2];

    const Vector3 e1 = t_v2 - t_v1;
    const Vector3 e2 = t_v3 - t_v1;
    
    const auto p = direction.cross(e2);

    const auto det = p.dot(e1);

    // if the ray is parallel to the triangle, there is no hit :)
    //~ if(det == approx(0.f)) return false;

    const auto t = origin - t_v1;

    const auto q = t.cross(e1);

    const auto u = simd::div_ps(p.dot(t), det);
    const auto v = simd::div_ps(q.dot(direction), det);

    // Scalar: if(u >= 0.f && v >= 0.f && u + v <= 1.f)
    auto zero = simd::setzero_ps();
    auto one = simd::set1_ps(1.f);
    auto MASK = simd::cmple_ps(zero, u);
    MASK = simd::and_ps(MASK, simd::cmple_ps(zero, v));
    MASK = simd::and_ps(MASK, simd::cmple_ps(simd::add_ps(u, v), one));
    
    bool_t result = simd::castps_si(MASK); // add all intersections to result
    
    auto distance = simd::and_ps(simd::div_ps(q.dot(e2), det), MASK);
    const auto max = simd::and_ps(simd::not_ps(MASK), max_distance());
    distance = simd::or_ps(distance, max); // We need to set all places that have no intersection to max

    // Negative intersections are intersections at infinity
    MASK = simd::cmplt_ps(distance, simd::setzero_ps());
    distance = simd::or_ps(simd::and_ps(simd::not_ps(MASK), distance), simd::and_ps(MASK, max_distance()));
    
    result = simd::castps_si(simd::and_ps(simd::castsi_ps(result), simd::not_ps(MASK))); // remove intersections with negative distance from result
    
    *out_distance = distance;
    return result;
  }
  
  inline bool_t intersectAABB(const AABox3 &aabb) const
  {
    auto aabb_min = location_t(aabb.min);
    auto aabb_max = location_t(aabb.max);

    // https://people.csail.mit.edu/amy/papers/box-jgt.pdf
    auto signx = simd::cmple_ps(dir_inv.x, simd::setzero_ps());
    auto signy = simd::cmple_ps(dir_inv.y, simd::setzero_ps());
    auto signz = simd::cmple_ps(dir_inv.z, simd::setzero_ps());

    using floatty = simd::floatty;
    auto mix = [](floatty mask, floatty x1, floatty x2) {
      return simd::or_ps(simd::and_ps(mask, x1), simd::and_ps(simd::not_ps(mask), x2));
    };

    auto tmin  = simd::mul_ps(simd::sub_ps(mix(signx, aabb_max.x, aabb_min.x), origin.x), dir_inv.x);
    auto tmax  = simd::mul_ps(simd::sub_ps(mix(signx, aabb_min.x, aabb_max.x), origin.x), dir_inv.x);
    auto tymin = simd::mul_ps(simd::sub_ps(mix(signy, aabb_max.y, aabb_min.y), origin.y), dir_inv.y);
    auto tymax = simd::mul_ps(simd::sub_ps(mix(signy, aabb_min.y, aabb_max.y), origin.y), dir_inv.y);

    // Scalar: if ((tmin > tymax) || (tymin > tmax)) return false;
    auto EARLY = simd::not_ps(simd::or_ps(simd::cmplt_ps(tymax, tmin), simd::cmplt_ps(tmax, tymin))); // TODO logical reordering of NOT
    // Scalar: if (tymin > tmin) tmin = tymin;
    auto MASK = simd::cmplt_ps(tmin, tymin);
    tmin = mix(MASK, tymin, tmin);
    // Scalar: if (tymax < tmax) tmax = tymax;
    MASK = simd::cmplt_ps(tymax, tmax);
    tmax = mix(MASK, tymax, tmax);

    auto tzmin = simd::mul_ps(simd::sub_ps(mix(signz, aabb_max.z, aabb_min.z), origin.z), dir_inv.z);
    auto tzmax = simd::mul_ps(simd::sub_ps(mix(signz, aabb_min.z, aabb_max.z), origin.z), dir_inv.z);

    // Scalar: if ((tmin > tzmax) || (tzmin > tmax)) return false;
    auto EARLY2 = simd::not_ps(simd::or_ps(simd::cmplt_ps(tzmax, tmin), simd::cmplt_ps(tmax, tzmin))); // TODO logical reordering of NOT
    // Scalar: if (tzmin > tmin) tmin = tzmin;
    MASK = simd::cmplt_ps(tmin, tzmin);
    tmin = mix(MASK, tzmin, tmin);
    // Scalar: if (tzmax < tmax) tmax = tzmax;
    MASK = simd::cmplt_ps(tzmax, tmax);
    tmax = mix(MASK, tzmax, tmax);

    auto result = simd::castsi_ps(simd::set1_epi32(-1));// simd::and_ps(simd::cmplt_ps(tmin, t1), simd::cmplt_ps(t0, tmax));
    return simd::castps_si(simd::and_ps(EARLY, simd::and_ps(EARLY2, result)));
  }

  location_t getIntersectionPoint(distance_t intersection_distance) const {
    return origin + direction * intersection_distance;
  }

private:
  SSERay(location_t origin, simd::Vec3Pack direction) : origin(origin), direction(direction), dir_inv(vec3_t(1.f, 1.f, 1.f) / direction) {}

  static distance_t getLambert(simd::Vec3Pack L, simd::Vec3Pack N, distance_t light_dist_squared) {
    return simd::div_ps(simd::max_ps(L.dot(N), simd::set1_ps(0.f)), light_dist_squared);
  }

public:
  static SSERay getShadowRay(Light light, location_t P, distance_t *ld) {
    const auto light_vector = location_t(light.position) - P;
		const auto light_distance = light_vector.length();
		const auto L = light_vector / light_distance;
    *ld = light_distance;
    return {P + L * simd::set1_ps(0.001f), L};
  }

  static color_t getMaterialColors(const Scene &scene, intersect_t intersects) {
    alignas(16) std::array<TriangleIndex, simd::REGISTER_CAPACITY_I32> int_ersects;
    simd::store_si((simd::intty *) &int_ersects[0], intersects);
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> R;
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> G;
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> B;
    for (unsigned i = 0; i < simd::REGISTER_CAPACITY_I32; ++i) {
      auto triangle = int_ersects[i];
      if (triangle != TriangleIndex_Invalid) {
        const auto material_index = scene.triangles[int_ersects[i]].material_index;
        const auto C = scene.materials[material_index].color;
        R[i] = C.r;
        G[i] = C.g;
        B[i] = C.b;
      } else {
        R[i] = 0;
        G[i] = 0;
        B[i] = 0;
      }
    }
    return {simd::load_ps(&R[0]), simd::load_ps(&G[0]), simd::load_ps(&B[0])};
  }

  static vec3_t getNormals(const Scene &scene, intersect_t intersects) {
    alignas(16) std::array<TriangleIndex, simd::REGISTER_CAPACITY_I32> int_ersects;
    simd::store_si((simd::intty *) &int_ersects[0], intersects);
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> X;
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> Y;
    alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> Z;
    for (unsigned i = 0; i < simd::REGISTER_CAPACITY_I32; ++i) {
      auto triangle = int_ersects[i];
      if (triangle != TriangleIndex_Invalid) {
        // TODO we could also load the triangles into a Vec3Pack and calculate normals with SIMD code
        const auto N = scene.triangles[triangle].calculateNormal();
        X[i] = N.x;
        Y[i] = N.y;
        Z[i] = N.z;
      } else {
        // TODO is this necessary or default initialized?
        X[i] = 0;
        Y[i] = 0;
        Z[i] = 0;
      }
    }
    return {simd::load_ps(&X[0]), simd::load_ps(&Y[0]), simd::load_ps(&Z[0])};
  }

  static color_t shade(const Scene &scene, const location_t &P, intersect_t intersects, const Light &light, distance_t intersection_distance, vec3_t N, color_t mat_colors) {
    // TODO duplicated code
    const auto light_vector = location_t(light.position) - P;
		const auto light_distance = light_vector.length();
		const auto L = light_vector / light_distance;

    const auto MASK = simd::cmplt_ps(light_distance, intersection_distance);

    const auto light_dist_squared = simd::mul_ps(light_distance, light_distance);
    const auto lambert = getLambert(L, N, light_dist_squared);
    
    const auto res = mat_colors * color_t(light.color) * lambert;
    return {simd::and_ps(MASK, res.x), simd::and_ps(MASK, res.y), simd::and_ps(MASK, res.z)};
  }

  static distance_t max_distance() {
    return simd::set1_ps(std::numeric_limits<float>::max());
  }

  static void updateIntersections(intersect_t *intersect, TriangleIndex triangle_index, distance_t *minimum_distance, distance_t distance) {
    const auto DIST_MASK = simd::cmplt_ps(distance, *minimum_distance);
    const auto DIST_MASK_INV = simd::not_ps(DIST_MASK);
    *minimum_distance = simd::or_ps(simd::and_ps(DIST_MASK, distance), simd::and_ps(DIST_MASK_INV, *minimum_distance));
    const auto trianglevec_float = simd::castsi_ps(simd::set1_epi32(triangle_index));
    const auto intersect_float = simd::castsi_ps(*intersect);
    *intersect = simd::castps_si(simd::or_ps(simd::and_ps(DIST_MASK, trianglevec_float), simd::and_ps(DIST_MASK_INV, intersect_float)));
  }

  static bool_t isNoIntersection(intersect_t intersect) {
    return simd::cmpeq_epi32(intersect, simd::set1_epi32(TriangleIndex_Invalid));
  }
  
  static intersect_t getNoIntersection() {
    return simd::set1_epi32(TriangleIndex_Invalid);
  }

  static inline bool isAll(bool_t b) {
    return simd::isAll(b);
  }

  static inline bool isAny(bool_t b) {
    return simd::isAny(b);
  }

  static inline bool_t isOppositeDirection(const vec3_t v1, const vec3_t v2) {
    return simd::castps_si(simd::cmple_ps(v1.dot(v2), simd::setzero_ps()));
  }
};

inline std::ostream &operator<<(std::ostream &o, const SSERay &r) {
  o << "Origin: " << r.origin << "\n" << "Direction: " << r.direction;
  return o;
}

inline void writeColorToImage(const SSEColor &c, ImageView &img, IntDimension2::dim_t x, IntDimension2::dim_t y) {
  alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> R;
  alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> G;
  alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> B;
  simd::store_ps(&R[0], c.x);
  simd::store_ps(&G[0], c.y);
  simd::store_ps(&B[0], c.z);
  for (unsigned i_y = 0; i_y < SSERay::dim.h && i_y + y < img.resolution.h; ++i_y) {
    for (unsigned i_x = 0; i_x < SSERay::dim.w && i_x + x < img.resolution.w; ++i_x) {
      auto idx = i_y * SSERay::dim.w + i_x;
      img.setPixel(x + i_x, y + i_y, {R[idx], G[idx], B[idx]});
    }
  }
}
