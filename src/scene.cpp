#include "scene.hpp"

Color Scene::trace(const Ray &ray)
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

	if(intersected_triangle != TriangleIndex_Invalid)
	{
		const MaterialIndex material_index = triangles[intersected_triangle].material_index;
		ASSERT(material_index != MaterialIndex_Invalid);
		return materials[material_index].color;
	}
	else
	{
		return background_color;
	}
}
