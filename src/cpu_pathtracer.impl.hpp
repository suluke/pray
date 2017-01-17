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

  alignas(32) std::array<TriangleIndex, simd::REGISTER_CAPACITY_I32> triangles;
  simd::store_si((simd::intty *) triangles.data(), intersect);

  // base for hemisphere sampling
  alignas(32) std::array<float, simd::REGISTER_CAPACITY_FLOAT> X_x, X_y, X_z;
  // material colors
  alignas(32) std::array<float, simd::REGISTER_CAPACITY_FLOAT> R, G, B;
  alignas(32) std::array<int32_t, simd::REGISTER_CAPACITY_I32> KillBuf;
  // load all the scattered information into consecutive vectors
  for (unsigned i = 0; i < triangles.size(); ++i) {
    auto T_idx = triangles[i];
    if (T_idx != TriangleIndex_Invalid) {
      auto &triangle = scene.triangles[T_idx];
      auto X = (triangle.vertices[1] - triangle.vertices[0]).normalize();
      X_x[i] = X.x;
      X_y[i] = X.y;
      X_z[i] = X.z;
      const auto &material = scene.materials[triangle.material_index];
      if (material.isEmission) {
	KillBuf[i] = true;
      } else {
	KillBuf[i] = false;
      }
      auto &c = material.color;
      R[i] = c.r;
      G[i] = c.g;
      B[i] = c.b;
    } else {
      X_x[i] = 0;
      X_y[i] = 0;
      X_z[i] = 0;
      R[i] = scene.background_color.r;
      G[i] = scene.background_color.g;
      B[i] = scene.background_color.b;
      KillBuf[i] = true;
    }
  }
  auto Killed = simd::load_si((simd::intty *) KillBuf.data());
  auto Colr = typename ray_t::color_t{simd::load_ps(R.data()), simd::load_ps(G.data()), simd::load_ps(B.data())};
  auto Alive = simd::castps_si(simd::and_ps(simd::not_ps(simd::castsi_ps(Killed)), simd::castsi_ps(mask)));

  // The Killed mask tells if we hit an emitting triangle (aka a light source)
  // Therefore, `Colr AND Killed` gives us all the emission colors
  if (!ray_t::isAny(Alive) || depth >= opts.max_depth)
    return typename ray_t::color_t {
      simd::and_ps(simd::castsi_ps(Killed), Colr.x),
      simd::and_ps(simd::castsi_ps(Killed), Colr.y),
      simd::and_ps(simd::castsi_ps(Killed), Colr.z)
    };

  const auto N = ray_t::getNormals(scene, intersect);
  auto X = typename ray_t::vec3_t{simd::load_ps(X_x.data()), simd::load_ps(X_y.data()), simd::load_ps(X_z.data())};
  auto Y = N.cross(X).normalize(); // this will be INFINITY (division by zero) for invalid triangles
  auto &Z = N;
  const auto P = ray.getIntersectionPoint(intersection_distance) + N * 0.0001f;

  typename ray_t::color_t value{0, 0, 0};
  for (unsigned i = 0; i < opts.num_samples; ++i) {
    ray_t next(P, sampleHemisphere(X, Y, Z));
    value += trace(scene, next, depth + 1, Alive);
  }
  auto MaterialColor = typename ray_t::color_t {
    simd::and_ps(simd::castsi_ps(Alive), Colr.x),
    simd::and_ps(simd::castsi_ps(Alive), Colr.y),
    simd::and_ps(simd::castsi_ps(Alive), Colr.z)
  };
  value = MaterialColor * value / opts.num_samples;
  return value;
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
