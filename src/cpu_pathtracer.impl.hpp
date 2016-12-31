template <class ray_t, class accel_t>
void CpuPathTracer<ray_t, accel_t>::render(ImageView &image) const {
  Vector3 left, right, bottom, top;
	const float aspect = (float) image.resolution.h / image.resolution.w;
	scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	float max_x = (float) image.resolution.w;
	float max_y = (float) image.img.resolution.h;

#ifdef WITH_OMP
	#pragma omp parallel for schedule(guided, 10)
#endif
	for(long y = 0; y < image.resolution.h; y += ray_t::dim::h) {
		for(long x = 0; x < image.resolution.w; x += ray_t::dim::w) {
			if (!subsampling_enabled || (
					x == 0 || x == image.resolution.w-1 || y == 0 || image.resolution.h -1 == y ||
					(x%2 == 0 && y%2 == 1) || (x%2 == 1 && y%2 == 0)))
			{
				ray_t ray(scene.camera, left, top, x, image.getGlobalY(y), max_x, max_y);
				auto c = trace(scene, ray);
				writeColorToImage(c, image, x, y);
			}
		}
	}
}

template <class ray_t, class accel_t>
typename ray_t::color_t CpuPathTracer<ray_t, accel_t>::trace(const PathScene &scene, const ray_t &ray, unsigned depth) const {
  typename ray_t::distance_t intersection_distance;
	const auto intersected_triangle = acceleration_structure.intersect(scene, ray, &intersection_distance);

  if (ray_t::isAll(ray_t::isNoIntersection(intersected_triangle)))
    return scene.background_color;

  const auto &triangle = scene.triangles[intersected_triangle];
  const auto material_index = triangle.material_index;
  ASSERT(material_index != MaterialIndex_Invalid);
  auto &material = scene.materials[material_index];
    
  if (depth >= opts.max_depth || material.isEmission)
    return material.color;

  const auto X = triangle.vertices[1] - triangle.vertices[0];
  const auto N = ray_t::getNormals(scene, intersected_triangle);
  const auto Y = X.cross(N);
  const auto P = ray.getIntersectionPoint(intersection_distance);

  Color value{0, 0, 0};
  for (unsigned i = 0; i < opts.num_samples; ++i) {
    ray_t next(P, sampleHemisphere(X, Y, N));
    value += trace(scene, next, depth + 1);
  }
  value = material.color * value / opts.num_samples;
  return value;
}


template <class ray_t, class accel_t>
typename ray_t::vec3_t CpuPathTracer<ray_t, accel_t>::sampleHemisphere(const typename ray_t::vec3_t &X, const typename ray_t::vec3_t &Y, const typename ray_t::vec3_t &Z) const {
  float u1 = sampling_rand();
  float u2 = sampling_rand();
  float r = std::sqrt(1 - u1);
  float phi = 2 * std::acos(-1) * u2;
  float x = std::cos(phi) * r;
  float y = std::sin(phi) * r;
  float z = std::sqrt(u1);
  return X * x + Y * y + Z * z;
}
