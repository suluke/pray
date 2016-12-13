#include "scene.hpp" // includes math and image

struct Ray {
  using dim_t = IntDimension2::dim_t;
  using intersect_t = TriangleIndex;
  using material_t = MaterialIndex;
  using color_t = Color;
  using location_t = Vector3;
  using distance_t = float;
  
  static constexpr IntDimension2 dim = {1, 1};
  
  const Vector3 origin;
  const Vector3 direction;

  Ray(const Camera &cam, const Vector3 &left, const Vector3 &top, const dim_t x, const dim_t y, float max_x, float max_y)
  : origin(cam.position), direction((top * (1.f - (2 * y + 1) / max_y) + left * (1.f - (2 * x + 1) / max_x) + cam.direction).normalize()) {}


  inline bool intersectTriangle(const Triangle &triangle, distance_t *out_distance) const
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

  location_t getIntersectionPoint(distance_t intersection_distance) const {
    return origin + direction * intersection_distance;
  }

private:
  Ray(location_t origin, Vector3 direction) : origin(origin), direction(direction) {}

public:
  static Ray getShadowRay(Light light, location_t P, distance_t *ld) {
    const Vector3 light_vector = light.position - P;
		const float light_distance = light_vector.length();
		const Vector3 L = light_vector / light_distance;
    *ld = light_distance;
    return {P + L * 0.001f, L};
  }
  
  static color_t shade(const Scene &scene, const location_t &P, intersect_t triangle, const Light &light, distance_t intersection_distance) {
    // TODO duplicated code
    const Vector3 light_vector = light.position - P;
		const float light_distance = light_vector.length();
		const Vector3 L = light_vector / light_distance;

    if (intersection_distance < light_distance)
      return {0.f, 0.f, 0.f};

    const auto N = scene.triangles[triangle].calculateNormal();
    const auto material_index = scene.triangles[triangle].material_index;
    ASSERT(material_index != MaterialIndex_Invalid);
    
    return scene.materials[material_index].color * light.color * (std::max(L.dot(N), 0.f) / (light_distance * light_distance));
  }

  static distance_t max_distance() {
    return std::numeric_limits<distance_t>::max();
  }

  static void updateIntersections(intersect_t *intersected_triangle, TriangleIndex triangle_index, distance_t *minimum_distance, distance_t distance) {
    if(distance < *minimum_distance)
    {
      *intersected_triangle = triangle_index;
      *minimum_distance = distance;
    }
  }
};

#include "simd.hpp"
#include <array>

struct SSEColor : public simd::Vec3Pack {
  SSEColor(const Color &c) : simd::Vec3Pack(c.r, c.g, c.b) {}
  SSEColor(const simd::Vec3Pack &c) {}
  SSEColor(simd::floatty r, simd::floatty g, simd::floatty b) : simd::Vec3Pack(r, g, b) {}
};

inline void writeColorToImage(const SSEColor &c, ImageView &img, IntDimension2::dim_t x, IntDimension2::dim_t y) {
	//img.setPixel(x, y, c);
}

struct SSEIntersect {
  SSEIntersect(TriangleIndex t) {}
  bool operator==(TriangleIndex t) const {
    // TODO
    return false;
  }
  simd::intty intersections;
};

struct SSERay {
  using dim_t = IntDimension2::dim_t;
  using intersect_t = SSEIntersect;
  using material_t = simd::floatty;
  using color_t = SSEColor;
  using location_t = simd::Vec3Pack;
  using distance_t = simd::floatty;

  static constexpr IntDimension2 dim = {simd::REGISTER_CAPACITY_FLOAT == 8 ? 4 : 2, 2};

  const location_t origin;
  simd::Vec3Pack direction;

  SSERay(const Camera &cam, const Vector3 &left, const Vector3 &top, const dim_t x, const dim_t y, float max_x, float max_y) : origin(cam.position) {
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> X;
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> Y;
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> Z;
    for (unsigned i = 0; i < simd::REGISTER_CAPACITY_FLOAT; ++i) {
      X[i] = cam.direction.x;
      Y[i] = cam.direction.y;
      Z[i] = cam.direction.z;
    }
    for (unsigned i_x = 0; i_x < dim.w; ++i_x) {
      float f_x = 1.f - (2 * (x + i_x) + 1) / max_x;
      auto l = left * f_x;
      for (unsigned i_y = 0; i_y < dim.h; ++i_y) {
        X[i_y * dim.w + i_x] = l.x;
        Y[i_y * dim.w + i_x] = l.y;
        Z[i_y * dim.w + i_x] = l.z;
      }
    }
    for (unsigned i_y = 0; i_y < dim.h; ++i_y) {
      float f_y = 1.f - (2 * (y + i_y) + 1) / max_y;
      auto t = top * f_y;
      for (unsigned i_x = 0; i_x < dim.h; ++i_x) {
        X[i_y * dim.w + i_x] = t.x;
        Y[i_y * dim.w + i_x] = t.y;
        Z[i_y * dim.w + i_x] = t.z;
      }
    }
    direction.x = simd::load_ps(&X[0]);
    direction.y = simd::load_ps(&Y[0]);
    direction.z = simd::load_ps(&Z[0]);
    direction.normalize();
  }

  inline bool intersectTriangle(const Triangle &triangle, distance_t *out_distance) const {
    // TODO
    return false;
  }

  location_t getIntersectionPoint(distance_t intersection_distance) const {
    return origin + direction * intersection_distance;
  }

private:
  SSERay(location_t origin, simd::Vec3Pack direction) : origin(origin), direction(direction) {}

  static simd::Vec3Pack getNormals(const Scene &scene, intersect_t intersects) {
    std::array<TriangleIndex, simd::REGISTER_CAPACITY_I32> int_ersects;
    simd::store_si((simd::intty *) &int_ersects[0], intersects.intersections);
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> X;
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> Y;
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> Z;
    for (unsigned i = 0; i < simd::REGISTER_CAPACITY_I32; ++i) {
      const auto N = scene.triangles[int_ersects[i]].calculateNormal();
      X[i] = N.x;
      Y[i] = N.y;
      Z[i] = N.z;
    }
    return {simd::load_ps(&X[0]), simd::load_ps(&Y[0]), simd::load_ps(&Z[0])};
  }

  static color_t getMaterialColors(const Scene &scene, intersect_t intersects) {
    std::array<TriangleIndex, simd::REGISTER_CAPACITY_I32> int_ersects;
    simd::store_si((simd::intty *) &int_ersects[0], intersects.intersections);
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> R;
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> G;
    std::array<float, simd::REGISTER_CAPACITY_FLOAT> B;
    for (unsigned i = 0; i < simd::REGISTER_CAPACITY_I32; ++i) {
      const auto material_index = scene.triangles[int_ersects[i]].material_index;
      const auto C = scene.materials[material_index].color;
      R[i] = C.r;
      G[i] = C.g;
      B[i] = C.b;
    }
    return {simd::load_ps(&R[0]), simd::load_ps(&G[0]), simd::load_ps(&B[0])};
  }

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

  static color_t shade(const Scene &scene, const location_t &P, intersect_t intersects, const Light &light, distance_t intersection_distance) {
    // TODO duplicated code
    const auto light_vector = location_t(light.position) - P;
		const auto light_distance = light_vector.length();
		const auto L = light_vector / light_distance;

    const auto MASK = simd::cmplt_ps(light_distance, intersection_distance);

    const auto N = getNormals(scene, intersects);
    const auto materialColors = getMaterialColors(scene, intersects);

    const auto light_dist_squared = light_distance * light_distance;
    const auto lambert = getLambert(L, N, light_dist_squared);
    
    const auto res = materialColors * color_t(light.color) * lambert;
    return {simd::and_ps(MASK, res.x), simd::and_ps(MASK, res.y), simd::and_ps(MASK, res.z)};
  }

  static distance_t max_distance() {
    return simd::set1_ps(std::numeric_limits<float>::max());
  }

  static void updateIntersections(intersect_t *intersectTriangle, TriangleIndex triangle_index, distance_t *minimum_distance, distance_t distance) {
    // TODO
  }
};
