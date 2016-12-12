#include "cpu_tracer.hpp"
#include "pray/Config.h"

template <class ray_t>
typename ray_t::intersect_t CpuTracer<ray_t>::intersectTriangle(const Scene &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const
{
	typename ray_t::intersect_t intersected_triangle = TriangleIndex_Invalid;
	typename ray_t::distance_t minimum_distance = ray_t::max_distance();

	for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(scene.triangles.size()); ++triangle_index)
	{
		const Triangle &triangle = scene.triangles[triangle_index];

		float distance;
		if(ray.intersectTriangle(triangle, &distance))
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
typename ray_t::color_t CpuTracer<ray_t>::trace(const Scene &scene, const ray_t &ray) const
{
	typename ray_t::distance_t intersection_distance;
	const typename ray_t::intersect_t intersected_triangle = intersectTriangle(scene, ray, &intersection_distance);

	if (intersected_triangle == TriangleIndex_Invalid) return scene.background_color;

	//~ // optimization: back faces are never lit
	//~ if(ray.direction.dot(N) >= 0.f) return Color(0.f, 0.f, 0.f);

	const auto P = ray.getIntersectionPoint(intersection_distance);

	typename ray_t::color_t result_color(0.f, 0.f, 0.f);

	for (auto &light : scene.lights)
	{
		typename ray_t::distance_t light_distance;
		const ray_t shadow_ray = ray_t::getShadowRay(light, P, &light_distance);
		if(intersectTriangle(scene, shadow_ray, &intersection_distance) == TriangleIndex_Invalid || intersection_distance > light_distance)
		{
			// don't use intersection_distance here, use light_distance instead!
			const typename ray_t::color_t shading_color = ray_t::shade(scene, P, intersected_triangle, light);
			result_color += shading_color;
		}
	}

	return result_color;
}

template <class ray_t>
void CpuTracer<ray_t>::render(ImageView &image) const
{
	Vector3 left, right, bottom, top;
	const float aspect = (float) image.resolution.h / image.resolution.w;
	scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

	float max_x = (float) image.resolution.w;
	float max_y = (float) image.img.resolution.h;
	#ifdef WITH_SUBSAMPLING
		const bool subsampling_enabled = true;
	#else
		const bool subsampling_enabled = false;
	#endif /*WITH_SUBSAMPLING*/

	#pragma omp parallel for
	for(long y = 0; y < image.resolution.h; y += ray_t::dim.h) {
		for(long x = 0; x < image.resolution.w; x += ray_t::dim.w) {
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
	if (subsampling_enabled) {
		#pragma omp parallel for
		for(long y = 1; y < image.resolution.h-1; ++y)
		{
			for(long x = 1; x < image.resolution.w-1; ++x)
			{
				if (x == 0 || x == image.resolution.w-1 || y == 0 || image.resolution.h -1 == y || (x%2 == 0 && y%2 == 1) || (x%2 == 1 && y%2 == 0)) {
					continue;
				} else {
					Color c = image.getPixel(x,y);
					c += image.getPixel(x-1,y) * 0.25f;
					c += image.getPixel(x,y-1) * 0.25f;
					c += image.getPixel(x+1,y) * 0.25f;
					c += image.getPixel(x,y+1) * 0.25f;
					image.setPixel(x, y, c);
				}
			}
		}
	}
}
