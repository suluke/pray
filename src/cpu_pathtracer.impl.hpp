#include "ray_ops.hpp"

template <class ray_t, class accel_t>
void CpuPathTracer<ray_t, accel_t>::render(ImageView &image) const {
  Vector3 left, right, bottom, top;
  const float aspect = (float) image.resolution.h / image.resolution.w;
  scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

  float max_x = (float) image.resolution.w;
  float max_y = (float) image.img.resolution.h;

#ifdef WITH_OMP
  #pragma omp parallel for schedule(guided, 10) collapse(2)
#endif
  for(long y = 0; y < image.resolution.h; y += ray_t::dim::h) {
    for(long x = 0; x < image.resolution.w; x += ray_t::dim::w) {
      ray_t ray(scene.camera, left, top, x, image.getGlobalY(y), max_x, max_y);
      auto c = trace(scene, ray);
      writeColorToImage(c, image, x, y);
    }
  }
}

template <class ray_t, class accel_t>
typename ray_t::color_t CpuPathTracer<ray_t, accel_t>::trace(const PathScene &scene, const ray_t &ray, unsigned depth, typename ray_t::mask_t mask) const {
  typename ray_t::distance_t intersection_distance;
  const auto intersected_triangle = acceleration_structure.intersect(scene, ray, &intersection_distance);

  if (ray_t::isAll(ray_t::isNoIntersection(intersected_triangle)))
    return scene.background_color;

  typename ray_t::color_t material_color;
  if (ray_t::isAll(ray_ops::getEmission(intersected_triangle, scene, &material_color)) || depth >= opts.max_depth)
    return material_color;

  auto N = ray_t::getNormals(scene, intersected_triangle);
  typename ray_t::vec3_t X, Y, Z;
  ray_ops::getIntersectionBase(N, intersected_triangle, scene, &X, &Y, &Z);
  const auto P = ray.getIntersectionPoint(intersection_distance) + N * 0.0001f;

  typename ray_t::color_t value{0, 0, 0};
  // TODO this can be parallelized. Maybe with a global task system for all casted rays?
  for (unsigned i = 0; i < opts.num_samples; ++i) {
    ray_t next = ray_ops::sampleHemisphere<PathScene>(P, X, Y, Z, sampling_rand);
    value += trace(scene, next, depth + 1, mask);
  }
  value = ray_t::getMaterialColors(scene, intersected_triangle) * value / opts.num_samples;
  return value;
}
