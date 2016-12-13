#ifndef PRAY_DUMMY_ACCELERATION_H
#define PRAY_DUMMY_ACCELERATION_H

template<class ray_t>
struct DummyAcceleration
{
	void build(const Scene &) {}

	TriangleIndex intersect(const Scene &scene, const ray_t &ray, float *out_distance) const
	{
		TriangleIndex intersected_triangle = TriangleIndex_Invalid;
		float minimum_distance = std::numeric_limits<float>::max();

		for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(scene.triangles.size()); ++triangle_index)
		{
			const Triangle &triangle = scene.triangles[triangle_index];

			typename ray_t::distance_t distance;
			if(ray.intersectTriangle(triangle, &distance))
			{
				ray_t::updateIntersections(&intersected_triangle, triangle_index, &minimum_distance, distance);
			}
		}

		*out_distance = minimum_distance;
		return intersected_triangle;
	}
};

#endif
