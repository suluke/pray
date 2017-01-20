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
