#include "scene.hpp"

Color Scene::trace(const Ray &ray) const
{
	TriangleIndex intersected_triangle = TriangleIndex_Invalid;
	float minimum_distance = std::numeric_limits<float>::max();

	for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(triangles.size()); ++triangle_index)
	{
		const Triangle &triangle = triangles[triangle_index];

		float distance;
		if(intersectRayTriangle(ray.origin, ray.direction, triangle.vertices[0], triangle.vertices[1], triangle.vertices[2], &distance))
		{
			ASSERT(distance >= 0.f);
			if(distance < minimum_distance)
			{
				intersected_triangle = triangle_index;
				minimum_distance = distance;
			}
		}
	}

	if(intersected_triangle == TriangleIndex_Invalid) return background_color;

	const Vector3 P = ray.origin + ray.direction * minimum_distance;
	const Vector3 _N = triangles[intersected_triangle].calculateNormal();
	//TODO: hack intersection with back-face
	const Vector3 N = _N.dot(ray.direction) < 0.f ? _N : -_N;

	const MaterialIndex material_index = triangles[intersected_triangle].material_index;
	ASSERT(material_index != MaterialIndex_Invalid);

	Color result_color(0.f, 0.f, 0.f);

	for(auto &light : lights)
	{
		const Vector3 L = (light.position - P).normalize();

		//TODO: shadow ray

		const Color shading_color = materials[material_index].color * light.color * std::max(L.dot(N), 0.f);
		result_color += shading_color;
	}

	return result_color;
}
