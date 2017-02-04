template <class ray_t, class accel_t>
typename Ray<PathScene>::color_t CpuPathTracer<ray_t, accel_t>::trace(const PathScene &scene, const Ray<PathScene> &ray, unsigned depth) const {
  typename ray_t::distance_t intersection_distance;
  const auto intersected_triangle = acceleration_structure.intersect(scene, ray, true, &intersection_distance);

  if (ray_t::isAll(ray_t::isNoIntersection(intersected_triangle)))
    return scene.background_color;

  const auto &triangle = scene.triangles[intersected_triangle];
  const auto material_index = triangle.material_index;
  ASSERT(material_index != MaterialIndex_Invalid);
  auto &material = scene.materials[material_index];

  if (material.isEmission)
    return material.color;
  if (depth >= opts.max_depth)
    return Color {0, 0, 0};

  const auto N = ray_t::getNormals(scene, intersected_triangle);
  const auto X = (triangle.vertices[1] - triangle.vertices[0]).normalize();
  const auto Y = N.cross(X).normalize();
  const auto &Z = N;
  const auto P = ray.getIntersectionPoint(intersection_distance) + N * 0.0001f;

  Color value{0, 0, 0};
  for (unsigned i = 0; i < opts.num_samples; ++i) {
    ray_t next(P, sampleHemisphere(X, Y, Z));
    value += trace(scene, next, depth + 1);
  }
  value = material.color * value / opts.num_samples;
  return value;
}

template <class rt, class accel_t>
typename SSERay<PathScene>::color_t CpuPathTracer< rt, accel_t >::trace(const PathScene &scene, const SSERay<PathScene> &ray, unsigned depth, typename SSERay<PathScene>::mask_t mask) const {
  using ray_t = SSERay<PathScene>;

  typename ray_t::distance_t intersection_distance;
  const auto intersect = acceleration_structure.intersect(scene, ray, mask, &intersection_distance);

  alignas(simd::REQUIRED_ALIGNMENT) std::array<TriangleIndex, simd::REGISTER_CAPACITY_I32> triangles;
  simd::store_si((simd::intty *) triangles.data(), intersect);

  alignas(simd::REQUIRED_ALIGNMENT) std::array<float, simd::REGISTER_CAPACITY_FLOAT> X_x, X_y, X_z;
  alignas(simd::REQUIRED_ALIGNMENT) std::array<float, simd::REGISTER_CAPACITY_FLOAT> R, G, B;
  alignas(simd::REQUIRED_ALIGNMENT) std::array<int32_t, simd::REGISTER_CAPACITY_I32> EmissionMask;

  for (unsigned i = 0; i < triangles.size(); ++i) {
    auto T_idx = triangles[i];
    if (T_idx != TriangleIndex_Invalid) {
      const auto &triangle = scene.triangles[T_idx];
      const auto X = (triangle.vertices[1] - triangle.vertices[0]).normalize();
      X_x[i] = X.x;
      X_y[i] = X.y;
      X_z[i] = X.z;
      const auto &material = scene.materials[triangle.material_index];
      R[i] = material.color.r;
      G[i] = material.color.g;
      B[i] = material.color.b;
      EmissionMask[i] = material.isEmission ? -1 : 0u;
    } else {
      X_x[i] = 0.f;
      X_y[i] = 0.f;
      X_z[i] = 0.f;
      R[i] = scene.background_color.r;
      G[i] = scene.background_color.g;
      B[i] = scene.background_color.b;
      EmissionMask[i] = -1;
    }
  }

  const auto killed = simd::load_si((simd::intty *) EmissionMask.data());
  const auto material_color = ray_t::color_t{simd::load_ps(R.data()), simd::load_ps(G.data()), simd::load_ps(B.data())};
  const auto emission = ray_t::color_t {
    simd::and_ps(material_color.x, simd::castsi_ps(killed)),
    simd::and_ps(material_color.y, simd::castsi_ps(killed)),
    simd::and_ps(material_color.z, simd::castsi_ps(killed))
  };

  if(ray_t::isAll(killed) || depth >= opts.max_depth) {
    return emission;
  }

  const auto N = ray_t::getNormals(scene, intersect);
  const auto X = typename ray_t::vec3_t{simd::load_ps(X_x.data()), simd::load_ps(X_y.data()), simd::load_ps(X_z.data())};
  const auto Y = N.cross(X).normalize(); // this will be INFINITY (division by zero) for invalid triangles
  const auto &Z = N;
  const auto P = ray.getIntersectionPoint(intersection_distance) + N * 0.0001f;

  const auto alive = simd::castps_si(simd::not_ps(simd::castsi_ps(killed)));

  typename ray_t::color_t value{0, 0, 0};
  for (unsigned i = 0; i < opts.num_samples; ++i) {
    ray_t next(P, sampleHemisphere(X, Y, Z));
    value += trace(scene, next, depth + 1, alive);
  }

  const auto irradiance = ray_t::color_t {
    simd::and_ps(value.x, simd::castsi_ps(alive)),
    simd::and_ps(value.y, simd::castsi_ps(alive)),
    simd::and_ps(value.z, simd::castsi_ps(alive))
  };

  return irradiance * material_color  / opts.num_samples + emission;
}


template <class ray_t, class accel_t>
typename ray_t::vec3_t CpuPathTracer<ray_t, accel_t>::sampleHemisphere(const typename ray_t::vec3_t &X, const typename ray_t::vec3_t &Y, const typename ray_t::vec3_t &Z) const {
#ifndef WITH_CHEATS
  return sampleHemisphereImpl(X, Y, Z);
}

template <class ray_t, class accel_t>
typename Ray<PathScene>::vec3_t
CpuPathTracer<ray_t, accel_t>::sampleHemisphereImpl(const typename Ray<PathScene>::vec3_t &X, const typename Ray<PathScene>::vec3_t &Y, const typename Ray<PathScene>::vec3_t &Z) const {
#endif
  float u1 = sampling_rand();
  float u2 = sampling_rand();
  float r = std::sqrt(1.f - u1);
  float phi = 2 * std::acos(-1.f) * u2;
  float x = std::cos(phi) * r;
  float y = std::sin(phi) * r;
  float z = std::sqrt(u1);
  return X * x + Y * y + Z * z;
}

#ifndef WITH_CHEATS
template <class ray_t, class accel_t>
typename SSERay<PathScene>::vec3_t
CpuPathTracer<ray_t, accel_t>::sampleHemisphereImpl(const typename SSERay<PathScene>::vec3_t &X, const typename SSERay<PathScene>::vec3_t &Y, const typename SSERay<PathScene>::vec3_t &Z) const {
  alignas(simd::REQUIRED_ALIGNMENT) std::array<float, simd::REGISTER_CAPACITY_FLOAT> U1, U2;
  for (unsigned i = 0; i < U1.size(); ++i) {
    U1[i] = sampling_rand();
    U2[i] = sampling_rand();
  }
  auto u1 = simd::load_ps(U1.data());
  auto u2 = simd::load_ps(U2.data());
  auto r = simd::sqrt_ps(simd::sub_ps(simd::set1_ps(1.f), u1));
  auto PI = simd::set1_ps(std::acos(-1.f));
  auto phi = simd::mul_ps(simd::set1_ps(2.f), simd::mul_ps(PI, u2));
  // TODO implement vectorized trigonometric functions ourselves 
  alignas(simd::REQUIRED_ALIGNMENT) std::array<float, simd::REGISTER_CAPACITY_FLOAT> PHI, SIN, COS;
  simd::store_ps(PHI.data(), phi);
  for (unsigned i = 0; i < SIN.size(); ++i) {
    SIN[i] = std::sin(PHI[i]);
    COS[i] = std::cos(PHI[i]);
  }
  auto x = simd::mul_ps(simd::load_ps(COS.data()), r);
  auto y = simd::mul_ps(simd::load_ps(SIN.data()), r);
  auto z = simd::sqrt_ps(u1);
  return X * x + Y * y + Z * z;
}
#endif
