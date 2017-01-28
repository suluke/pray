typename Ray<PathScene>::color_t CudaPathTracer::trace(const PathScene &scene, const Ray<PathScene> &ray, unsigned depth) const {
  typename ray_t::distance_t intersection_distance;

  Color value{0, 0, 0};
  return value;
}

typename Ray<PathScene>::vec3_t CudaPathTracer::sampleHemisphere(const typename Ray<PathScene>::vec3_t &X, const typename Ray<PathScene>::vec3_t &Y, const typename Ray<PathScene>::vec3_t &Z) const {
  float u1 = sampling_rand();
  float u2 = sampling_rand();
  float r = std::sqrt(1.f - u1);
  float phi = 2 * std::acos(-1.f) * u2;
  float x = std::cos(phi) * r;
  float y = std::sin(phi) * r;
  float z = std::sqrt(u1);
  return X * x + Y * y + Z * z;
}
