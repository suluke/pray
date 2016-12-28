#ifndef PRAY_DUMMY_ACCELERATION_H
#define PRAY_DUMMY_ACCELERATION_H

template<class ray_t, class scene_t>
struct DummyAcceleration
{
	void build(const scene_t &) {}

	typename ray_t::intersect_t intersect(const scene_t &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const
	{
		typename ray_t::intersect_t intersected_triangle = ray_t::getNoIntersection();
		auto minimum_distance = ray_t::max_distance();

		for(TriangleIndex triangle_index = 0u; triangle_index < index_cast<TriangleIndex>(scene.triangles.size()); ++triangle_index)
		{
			const Triangle &triangle = scene.triangles[triangle_index];

			typename ray_t::distance_t distance;
			if(ray_t::isAny(ray.intersectTriangle(triangle, &distance)))
			{
				ray_t::updateIntersections(&intersected_triangle, triangle_index, &minimum_distance, distance);
			}
		}

		*out_distance = minimum_distance;
		return intersected_triangle;
	}
};

#endif
