#include "cpu_tracer.hpp"
#include "pray/Config.h"

template <class ray_t, class accel_t>
typename ray_t::color_t CpuTracer<ray_t, accel_t>::trace(const WhittedScene &scene, const ray_t &ray) const
{
	typename ray_t::distance_t intersection_distance;
	const auto intersected_triangle = acceleration_structure.intersect(scene, ray, &intersection_distance);

	if (ray_t::isAll(ray_t::isNoIntersection(intersected_triangle))) return scene.background_color;

	// optimization: back faces are never lit
	const auto N = ray_t::getNormals(scene, intersected_triangle);
#ifndef WITH_SSE
	// TODO We have a problem with normals being 0 here for no-hit SSE rays
	// Would it be ok if we returned BG-Color here? 
	if(!ray_t::isAny(ray_t::isOppositeDirection(ray.direction, N))) return Color(0.f, 0.f, 0.f);
#endif

	const auto P = ray.getIntersectionPoint(intersection_distance);

	const auto mat_colors = ray_t::getMaterialColors(scene, intersected_triangle);

	typename ray_t::color_t result_color(Color(0.f, 0.f, 0.f));

	for (auto &light : scene.lights)
	{
		typename ray_t::distance_t light_distance;
		const ray_t shadow_ray = ray_t::getShadowRay(light, P, &light_distance);
		// optimization: don't consider lights behind the triangle
		if (ray_t::isAll(ray_t::isOppositeDirection(shadow_ray.direction, N))) continue;
		// This check does not work because shadow rays are actually allowed to intersect triangles
		// - just not those BETWEEN hitpoint and light. See `MASK` in SSERay::shade
		/* const auto shadow_intersect = */ acceleration_structure.intersect(scene, shadow_ray, &intersection_distance);
		//~ if (ray_t::isAny(ray_t::isNoIntersection(shadow_intersect)))
		{
			const typename ray_t::color_t shading_color = ray_t::shade(scene, P, intersected_triangle, light, intersection_distance, N, mat_colors);
			result_color += shading_color;
		}
	}
	// TODO Although this is a no-op for scalar rays, this seems expensive for SSERays
	ray_t::addBackgroundcolor(result_color, intersected_triangle, scene.background_color);

	return result_color;
}

template <class ray_t, class accel_t>
void CpuTracer<ray_t, accel_t>::render(ImageView &image) const
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
	if (subsampling_enabled) {
#ifdef WITH_OMP
	#pragma omp parallel for
#endif
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
