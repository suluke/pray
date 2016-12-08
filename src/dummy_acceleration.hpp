#ifndef PRAY_DUMMY_ACCELERATION_H
#define PRAY_DUMMY_ACCELERATION_H

struct DummyAcceleration
{
	void build(const Scene &) {}

	TriangleIndex intersectTriangle(const Scene &scene, const Ray &ray, float *out_distance) const
	{
		TriangleIndex intersected_triangle = TriangleIndex_Invalid;
		float minimum_distance = std::numeric_limits<float>::max();

		for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(scene.triangles.size()); ++triangle_index)
		{
			const Triangle &triangle = scene.triangles[triangle_index];

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
};

#endif
