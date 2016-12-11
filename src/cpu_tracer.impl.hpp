#include "cpu_tracer.hpp"

template <class ray_t>
TriangleIndex CpuTracer<ray_t>::intersectTriangle(const Scene &scene, const ray_t &ray, float *out_distance) const
{
	TriangleIndex intersected_triangle = TriangleIndex_Invalid;
	float minimum_distance = std::numeric_limits<float>::max();

	for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(scene.triangles.size()); ++triangle_index)
	{
		const Triangle &triangle = scene.triangles[triangle_index];

		float distance;
		if(ray.intersectTriangle(triangle.vertices[0], triangle.vertices[1], triangle.vertices[2], &distance))
		{
			if(distance < minimum_distance)
			{
				intersected_triangle = triangle_index;
				minimum_distance = distance;
			}
		}
	}

	*out_distance = minimum_distance;
	return intersected_triangle;
}

template <class ray_t>
Color CpuTracer<ray_t>::trace(const Scene &scene, const ray_t &ray) const
{
	float intersection_distance;
	const TriangleIndex intersected_triangle = intersectTriangle(scene, ray, &intersection_distance);

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

		const ray_t shadow_ray(P + L * 0.001f, L);
		if(intersectTriangle(scene, shadow_ray, &intersection_distance) == TriangleIndex_Invalid || intersection_distance > light_distance)
		{
			// don't use intersection_distance here, use light_distance instead!
			const Color shading_color = scene.materials[material_index].color * light.color * (std::max(L.dot(N), 0.f) / (light_distance * light_distance));
			result_color += shading_color;
		}
	}

	return result_color;
}

template <class ray_t>
void CpuTracer<ray_t>::render(ImageView &image) const
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
#ifdef ADAPTIVE_SAMPLE
			if (x == 0 || x == image.resolution.w-1 || y == 0 || image.resolution.h -1 == y || (x%2 == 0 && y%2 == 1) || (x%2 == 1 && y%2 == 0)) {
#endif /*ADAPTIVE_SAMPE*/
				const float i_x = 1.f - (2 * x + 1) / max_x;

				ray_t ray(scene.camera.position, (left * i_x + top * i_y + scene.camera.direction).normalize());

				Color c = trace(scene, ray);
				image.setPixel(x, y, c);
#ifdef ADAPTIVE_SAMPLE
			}
#endif /*ADAPTIVE_SAMPLE*/
		}
	}
#ifdef ADAPTIVE_SAMPLE
#ifdef _MSC_VER // msvc does not support uint32_t as index variable in for loop
	#pragma omp parallel for
	for(intmax_t y = 1; y < image.resolution.h-1; ++y)
#else
	#pragma omp parallel for
	for(uint32_t y = 1; y < image.resolution.h-1; ++y)
#endif
	{
		for(uint32_t x = 1; x < image.resolution.w-1; ++x)
		{
			if (x == 0 || x == image.resolution.w-1 || y == 0 || image.resolution.h -1 == y || (x%2 == 0 && y%2 == 1) || (x%2 == 1 && y%2 == 0)) {
				continue;
			} else {
				Color c = image.getPixel(x,y);
				c += image.getPixel(x-1,y) * 0.25f;
				c += image.getPixel(x,y-1) * 0.25f;
				c += image.getPixel(x+1,y) * 0.25f;
				c += image.getPixel(x,y+1) * 0.25f;
				image.setPixel(x,y,c);
			}
		}
	}
#endif /*ADAPTIVE_SAMPLE*/
}
