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
typename Ray<PathScene>::color_t CpuPathTracer<ray_t, accel_t>::trace(const PathScene &scene, const Ray<PathScene> &ray, unsigned depth) const {
  typename ray_t::distance_t intersection_distance;
  const auto intersected_triangle = acceleration_structure.intersect(scene, ray, &intersection_distance);

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
  const auto intersect = acceleration_structure.intersect(scene, ray, &intersection_distance);

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

  const auto killed = simd::load_ps(reinterpret_cast<float*>(EmissionMask.data()));
  const auto material_color = ray_t::color_t{simd::load_ps(R.data()), simd::load_ps(G.data()), simd::load_ps(B.data())};
  const auto emission = ray_t::color_t{simd::and_ps(material_color.x, killed), simd::and_ps(material_color.y, killed), simd::and_ps(material_color.z, killed)};

  if(ray_t::isAll(killed) || depth >= opts.max_depth) {
    return emission;
  }

  const auto N = ray_t::getNormals(scene, intersect);
  const auto X = typename ray_t::vec3_t{simd::load_ps(X_x.data()), simd::load_ps(X_y.data()), simd::load_ps(X_z.data())};
  const auto Y = N.cross(X).normalize(); // this will be INFINITY (division by zero) for invalid triangles
  const auto &Z = N;
  const auto P = ray.getIntersectionPoint(intersection_distance) + N * 0.0001f;

  const auto alive = simd::not_ps(killed);

  typename ray_t::color_t value{0, 0, 0};
  for (unsigned i = 0; i < opts.num_samples; ++i) {
    ray_t next(P, sampleHemisphere(X, Y, Z));
    value += trace(scene, next, depth + 1, alive);
  }

  const auto irradiance = ray_t::color_t{simd::and_ps(value.x, alive), simd::and_ps(value.y, alive), simd::and_ps(value.z, alive)};

  return irradiance * material_color  / opts.num_samples + emission;
}


template <class ray_t, class accel_t>
typename ray_t::vec3_t CpuPathTracer<ray_t, accel_t>::sampleHemisphere(const typename ray_t::vec3_t &X, const typename ray_t::vec3_t &Y, const typename ray_t::vec3_t &Z) const {
  float u1 = sampling_rand();
  float u2 = sampling_rand();
  float r = std::sqrt(1.f - u1);
  float phi = 2 * std::acos(-1.f) * u2;
  float x = std::cos(phi) * r;
  float y = std::sin(phi) * r;
  float z = std::sqrt(u1);
  return X * x + Y * y + Z * z;
}
