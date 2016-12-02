#include "scene.hpp"

TriangleIndex Scene::intersectTriangle(const Ray &ray, float *out_distance) const
{
	TriangleIndex intersected_triangle = TriangleIndex_Invalid;
	float minimum_distance = std::numeric_limits<float>::max();

	for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(triangles.size()); ++triangle_index)
	{
		const Triangle &triangle = triangles[triangle_index];

		float distance;
		if(intersectRayTriangle(ray.origin, ray.direction, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2], &distance))
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

Color Scene::trace(const Ray &ray) const
{
	float intersection_distance;
	const TriangleIndex intersected_triangle = intersectTriangle(ray, &intersection_distance);

	if(intersected_triangle == TriangleIndex_Invalid) return background_color;

	const Vector3 P = ray.origin + ray.direction * intersection_distance;
	const Vector3 _N = triangles[intersected_triangle].calculateNormal();

	//TODO: Hack intersection with back-face. Remove this, we only need front-face lighting!
	const Vector3 N = _N.dot(ray.direction) < 0.f ? _N : -_N;

	const MaterialIndex material_index = triangles[intersected_triangle].material_index;
	ASSERT(material_index != MaterialIndex_Invalid);

	Color result_color(0.f, 0.f, 0.f);

	for(auto &light : lights)
	{
		const Vector3 light_vector = light.position - P;
		const float light_distance = light_vector.length();
		const Vector3 L = light_vector / light_distance;

		const Ray shadow_ray(P + L * 0.001f, L);
		if(intersectTriangle(shadow_ray, &intersection_distance) == TriangleIndex_Invalid || intersection_distance > light_distance)
		{
			// don't use intersection_distance here, use light_distance instead!
			const Color shading_color = materials[material_index].color * light.color * (std::max(L.dot(N), 0.f) / sqrt(light_distance));
			result_color += shading_color;
		}
	}

	return result_color;
}
