#include "cpu_tracer.hpp"

template<class AccelerationStructure>
inline void CpuTracer<AccelerationStructure>::preprocess()
{
	acceleration_structure.build(scene);
}

template<class AccelerationStructure>
inline Color trace(const Scene &scene, const AccelerationStructure &acceleration_structure, const Ray &ray)
{
	float intersection_distance;
	const TriangleIndex intersected_triangle = acceleration_structure.intersectTriangle(scene, ray, &intersection_distance);

	if(intersected_triangle == TriangleIndex_Invalid) return scene.background_color;

	const Vector3 N = scene.triangles[intersected_triangle].calculateNormal();

	// optimization: back faces are never lit
	if(ray.direction.dot(N) >= 0.f) return Color(0.f, 0.f, 0.f);

	const Vector3 P = ray.origin + ray.direction * intersection_distance;

	const MaterialIndex material_index = scene.triangles[intersected_triangle].material_index;
	ASSERT(material_index != MaterialIndex_Invalid);

	Color result_color(0.f, 0.f, 0.f);

	for(auto &light : scene.lights)
	{
		const Vector3 light_vector = light.position - P;
		const float light_distance = light_vector.length();
		const Vector3 L = light_vector / light_distance;

		const Ray shadow_ray(P + L * 0.001f, L);
		if(acceleration_structure.intersectTriangle(scene, shadow_ray, &intersection_distance) == TriangleIndex_Invalid || intersection_distance > light_distance)
		{
			// don't use intersection_distance here, use light_distance instead!
			const Color shading_color = scene.materials[material_index].color * light.color * (std::max(L.dot(N), 0.f) / (light_distance * light_distance));
			result_color += shading_color;
		}
	}

	return result_color;
}

template<class AccelerationStructure>
inline void CpuTracer<AccelerationStructure>::render(ImageView &image) const
{
	Vector3 left, right, bottom, top;
	// assuming fov is in x direction, otherwise invert this
	const float aspect = (float)image.resolution.h / image.resolution.w;
	scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	float max_x = (float) image.resolution.w;
	float max_y = (float) image.img.resolution.h;

#ifdef _MSC_VER // msvc does not support uint32_t as index variable in for loop
	#pragma omp parallel for
	for(intmax_t y = 0; y < image.resolution.h; ++y)
#else
	#pragma omp parallel for
	for(uint32_t y = 0; y < image.resolution.h; ++y)
#endif
	{
		const float i_y = 1.f - (2 * image.getGlobalY(y) + 1) / max_y;
		for(uint32_t x = 0; x < image.resolution.w; ++x)
		{
			const float i_x = 1.f - (2 * x + 1) / max_x;

			Ray ray(scene.camera.position, (left * i_x + top * i_y + scene.camera.direction).normalize());

			Color c = trace(scene, acceleration_structure, ray);
			image.setPixel(x, y, c);
		}
	}
}
