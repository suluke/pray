#include "pray/Config.h"

template<class bih_t>
struct BihBuilder
{
	const Scene &scene;
	bih_t &bih;

	struct TriangleData
	{
		AABox3 aabb;
		Vector3 centroid;

		TriangleData(const AABox3 &aabb, const Vector3 &centroid) : aabb(aabb), centroid(centroid) {}
	};
	std::vector<TriangleData> triangle_data; // each element corresponds to an element in scene.triangles

	typedef std::vector<TriangleIndex>::iterator TrianglesIt;

	BihBuilder(const Scene &scene, bih_t &bih) : scene(scene), bih(bih) {}

	void build()
	{
		bih.scene_aabb.clear();
		triangle_data.reserve(scene.triangles.size());
		for(const auto &t : scene.triangles)
		{
			auto aabb = t.calculateAabb();
			bih.scene_aabb.insert(aabb);
			triangle_data.emplace_back(aabb, t.calculateCentroid());
		}

		bih.triangles.resize(scene.triangles.size());
		for(TriangleIndex i=0u; i<scene.triangles.size(); ++i) bih.triangles[i] = i;

		// https://de.wikipedia.org/wiki/Bin%C3%A4rbaum#Abz.C3.A4hlungen
		// The bih is a binary tree with (at most) scene.triangles.size() leaves.
		bih.nodes.reserve(scene.triangles.size() + scene.triangles.size() - 1);

		bih.nodes.emplace_back();
#ifdef DEBUG
		bih.nodes.back().parent = nullptr;
#endif
		buildNode(bih.nodes.back(), bih.scene_aabb, bih.triangles.begin(), bih.triangles.end());
	}

	void buildNode(typename bih_t::Node &current_node, const AABox3 &initial_aabb, const TrianglesIt triangles_begin, const TrianglesIt triangles_end, const unsigned retry_depth = 0u)
	{
		ASSERT(initial_aabb.isValid());

#ifdef DEBUG
		// for conditional breakpoints
		auto node_index = &current_node - &bih.nodes[0];
#endif

		const auto children_count = std::distance(triangles_begin, triangles_end);

		const bool leaf_children_count_reached = children_count <= 4u; //TODO: how to pass this constant as a parameter? (pls not template...)

		/* retry_depth counts how often we retried to split a node with a smaller aabb. If is is too great (magic number), the triangles are very close to each
		   other and a split will most likely never be found (due to float inprecision). The speedup wouldn't be to great anyways... */
		const bool maximum_retry_depth_reached = retry_depth == 16u; //TODO: tweak this parameter? or find a better criterium (floating point inaccuracy reached (nextafter(aabb.min, +1.f) == aabb.max or something like that...))?

		if(leaf_children_count_reached || maximum_retry_depth_reached)
		{
			// build a leaf

			const auto children_index = std::distance(bih.triangles.begin(), triangles_begin);
			current_node.makeLeafNode(children_index, children_count);

#ifdef DEBUG
			current_node.child1 = current_node.child2 = nullptr;
#endif
		}
		else
		{
			// build a node

			const auto split_axis = getSplitAxis(initial_aabb);

			float left_plane = std::numeric_limits<float>::lowest(), right_plane = std::numeric_limits<float>::max();

			const auto pivot = (initial_aabb.min[split_axis] + initial_aabb.max[split_axis]) / 2.f;
			const auto split_element = std::partition(triangles_begin, triangles_end,
			[&](TriangleIndex t)
			{
				const bool left = triangle_data[t].centroid[split_axis] <= pivot;

				if(left) left_plane = std::max(left_plane, triangle_data[t].aabb.max[split_axis]);
				else right_plane = std::min(right_plane, triangle_data[t].aabb.min[split_axis]);

				return left;
			});

			if(split_element == triangles_begin || split_element == triangles_end)
			{
				// one side of the partition is empty, so retry with a smaller aabb

				AABox3 new_initial_aabb = initial_aabb;
				if(split_element == triangles_begin) new_initial_aabb.min[split_axis] = pivot;
				else                                 new_initial_aabb.max[split_axis] = pivot;

				return buildNode(current_node, new_initial_aabb, triangles_begin, triangles_end, retry_depth + 1u);
			}

			// allocate child nodes (this is a critical section)
			const auto children_index = bih.nodes.size();
			ASSERT(bih.nodes.capacity() - bih.nodes.size() >= 2u); // we don't want relocation (breaks references)
			bih.nodes.emplace_back(); bih.nodes.emplace_back();

			current_node.makeSplitNode(split_axis, children_index, left_plane, right_plane);

			auto &child1 = bih.nodes[children_index+0];
			auto &child2 = bih.nodes[children_index+1];

#ifdef DEBUG
			current_node.index = node_index;
			current_node.split_point = pivot;
			current_node.child1 = &child1;
			current_node.child2 = &child2;
			child1.parent = child2.parent = &current_node;
#endif

			AABox3 child1_initial_aabb = initial_aabb, child2_initial_aabb = initial_aabb;
			child1_initial_aabb.max[split_axis] = pivot; // we could also insert the calculated planes here, let's try sometime whether this works better
			child2_initial_aabb.min[split_axis] = pivot;

			// recursion
			buildNode(child1, child1_initial_aabb, triangles_begin, split_element);
			buildNode(child2, child2_initial_aabb, split_element, triangles_end);
		}
	}

	unsigned getSplitAxis(const AABox3 &aabb)
	{
		auto dx = aabb.max.x - aabb.min.x;
		auto dy = aabb.max.y - aabb.min.y;
		auto dz = aabb.max.z - aabb.min.z;
		if(dx >= dy && dx >= dz) return 0u;
		else if(dy >= dz) return 1u;
		else return 2u;
	}
};

template<class ray_t>
void Bih<ray_t>::build(const Scene &scene)
{
	BihBuilder<Bih<ray_t>> builder(scene, *this);
	builder.build();
}

#ifdef DEBUG_TOOL
extern std::vector<size_t> bih_intersected_nodes;
#endif

// move to global namespace and use this intead of float *out_distance?
template<class ray_t>
struct IntersectionResult
{
	typename ray_t::intersect_t triangle = ray_t::getNoIntersection();
	typename ray_t::distance_t distance = ray_t::max_distance();
};

#ifndef WITH_BIH_SMART_TRAVERSION

// old algorithm!!!!!
template<class ray_t>
static void intersectBihNode(const typename Bih<ray_t>::Node &node, const AABox3 aabb, const Bih<ray_t> &bih, const Scene &scene, const ray_t &ray, IntersectionResult<ray_t> &intersection)
{
#ifdef DEBUG_TOOL
	bih_intersected_nodes.push_back(&node - &bih.nodes[0]);
#endif

	if(node.getType() == Bih<ray_t>::Node::Leaf)
	{
		for(unsigned i = 0u; i < node.getLeafData().children_count; ++i)
		{
			//TODO: remove double indirection (reorder scene.triangles)
			const TriangleIndex triangle_index = bih.triangles[node.getChildrenIndex() + i];
			const Triangle &triangle = scene.triangles[triangle_index];
      
			typename ray_t::distance_t distance;
			if(ray_t::isAny(ray.intersectTriangle(triangle, &distance)))
			{
				ray_t::updateIntersections(&intersection.triangle, triangle_index, &intersection.distance, distance);
			}
		}
	}
	else
	{
		const auto split_axis = node.getType();

		const auto &left_child = bih.nodes[node.getChildrenIndex()+0];
		const auto &right_child = bih.nodes[node.getChildrenIndex()+1];

		const auto left_plane = node.getSplitData().left_plane;
		const auto right_plane = node.getSplitData().right_plane;

		AABox3 left_aabb = aabb, right_aabb = aabb;
		left_aabb.max[split_axis] = left_plane;
		right_aabb.min[split_axis] = right_plane;

		// if ray_t are several rays and at least one ray has an intersection, the complete pack is checked

		//TODO: this can be done smarter!
		const auto intersect_left = ray.intersectAABB(left_aabb);
		if(ray_t::isAny(intersect_left)) intersectBihNode(left_child, left_aabb, bih, scene, ray, intersection);
    
		const auto intersect_right = ray.intersectAABB(right_aabb);
		if(ray_t::isAny(intersect_right)) intersectBihNode(right_child, right_aabb, bih, scene, ray, intersection);
	}
}

// old algorithm!!!!!
template<class ray_t>
typename ray_t::intersect_t Bih<ray_t>::intersect(const Scene &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const
{
#ifdef DEBUG_TOOL
	bih_intersected_nodes.clear();
#endif

	if(!ray_t::isAny(ray.intersectAABB(scene_aabb))) return ray_t::getNoIntersection();

	IntersectionResult<ray_t> intersection_result;
	intersectBihNode(nodes[0u], scene_aabb, *this, scene, ray, intersection_result);

	*out_distance = intersection_result.distance;
	return intersection_result.triangle;
}

#else

template<class ray_t>
static void intersectBihNode(const typename Bih<ray_t>::Node &node, const AABox3 aabb, const Bih<ray_t> &bih, const Scene &scene, const ray_t &ray, const Vector3 &direction_sign, const std::array<bool, 3> &direction_sign_equal, const typename ray_t::bool_t active_mask, IntersectionResult<ray_t> &intersection)
{
#ifdef DEBUG_TOOL
	bih_intersected_nodes.push_back(&node - &bih.nodes[0]);
#endif

	if(node.getType() == Bih<ray_t>::Node::Leaf)
	{
		for(unsigned i = 0u; i < node.getLeafData().children_count; ++i)
		{
			//TODO: remove double indirection (reorder scene.triangles)
			const TriangleIndex triangle_index = bih.triangles[node.getChildrenIndex() + i];
			const Triangle &triangle = scene.triangles[triangle_index];

			typename ray_t::distance_t distance;
			const auto intersected = ray_t::booleanAnd(ray.intersectTriangle(triangle, &distance), active_mask);

			if(ray_t::isAny(intersected))
			{
				// no need to pass active_mask here, updateIntersection handles this correctly
				// however, should ray_t ever get wider than one register, adding the mask might improve the performance if used correctly
				ray_t::updateIntersections(&intersection.triangle, triangle_index, &intersection.distance, distance);
			}
		}
	}
	else
	{
		const auto split_axis = node.getType();

		const auto &left_child = bih.nodes[node.getChildrenIndex()+0];
		const auto &right_child = bih.nodes[node.getChildrenIndex()+1];

		const auto left_plane = node.getSplitData().left_plane;
		const auto right_plane = node.getSplitData().right_plane;

		auto left_aabb = aabb;
		left_aabb.max[split_axis] = left_plane;
		auto right_aabb = aabb;
		right_aabb.min[split_axis] = right_plane;

		// The code in both cases is duplicated but with swapped left/right parameters. This gave a speedup of ~800ms on my machine with the sponza scene, no sse (lukasf)!
		if(direction_sign[split_axis] >= 0.f)
		{
			const auto intersect_first_mask = ray_t::booleanAnd(ray.intersectAABB(left_aabb), active_mask);
			if(ray_t::isAny(intersect_first_mask))
			{
				intersectBihNode(left_child, left_aabb, bih, scene, ray, direction_sign, direction_sign_equal, intersect_first_mask, intersection);
			}

			// using here that intersection.distance is default-initialized with ray_t::max_distance()
			const auto intersect_right_plane = ray.intersectAxisPlane(right_plane, split_axis, intersection.distance);

			if(ray_t::isAny(ray_t::booleanAnd(intersect_right_plane, active_mask)) || !direction_sign_equal[split_axis])
			{
				const auto intersect_second_mask = ray_t::booleanAnd(ray.intersectAABB(right_aabb), active_mask);
				if(ray_t::isAny(intersect_second_mask)) intersectBihNode(right_child, right_aabb, bih, scene, ray, direction_sign, direction_sign_equal, intersect_second_mask, intersection);
			}
		}
		else
		{
			const auto intersect_first_mask = ray_t::booleanAnd(ray.intersectAABB(right_aabb), active_mask);
			if(ray_t::isAny(intersect_first_mask))
			{
				intersectBihNode(right_child, right_aabb, bih, scene, ray, direction_sign, direction_sign_equal, intersect_first_mask, intersection);
			}

			// using here that intersection.distance is default-initialized with ray_t::max_distance()
			const auto intersect_left_plane = ray.intersectAxisPlane(left_plane, split_axis, intersection.distance);

			if(ray_t::isAny(ray_t::booleanAnd(intersect_left_plane, active_mask)) || !direction_sign_equal[split_axis])
			{
				const auto intersect_second_mask = ray_t::booleanAnd(ray.intersectAABB(left_aabb), active_mask);
				if(ray_t::isAny(intersect_second_mask)) intersectBihNode(left_child, left_aabb, bih, scene, ray, direction_sign, direction_sign_equal, intersect_second_mask, intersection);
			}
		}
	}
}

template<class ray_t>
typename ray_t::intersect_t Bih<ray_t>::intersect(const Scene &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const
{
#ifdef DEBUG_TOOL
	bih_intersected_nodes.clear();
#endif

	const auto active_mask = ray.intersectAABB(scene_aabb);
	if(!ray_t::isAny(active_mask)) return ray_t::getNoIntersection();

	const Vector3 direction_sign = ray.getSubrayDirection(ray_t::subrays_count / 2).sign();

	std::array<bool, 3> direction_sign_equal;
	ray.isDirectionSignEqualForAllSubrays(direction_sign, &direction_sign_equal);

	IntersectionResult<ray_t> intersection_result;
	intersectBihNode(nodes[0u], scene_aabb, *this, scene, ray, direction_sign, direction_sign_equal, active_mask, intersection_result);

	*out_distance = intersection_result.distance;
	return intersection_result.triangle;
}

#endif // WITH_BIH_NEW_TRAVERSION

template<class ray_t>
void Bih<ray_t>::printAnalsyis() const
{
	size_t inner_nodes_count = 0u, non_overlapping_inner_nodes_count = 0u, leaves_count = 0u, max_leaf_children_count = 0u;

	for(auto &n : nodes)
	{
		if(n.getType() == Node::Leaf)
		{
			++leaves_count;
			max_leaf_children_count = std::max<size_t>(max_leaf_children_count, n.getLeafData().children_count);
		}
		else
		{
			++inner_nodes_count;
			if(n.getSplitData().left_plane < n.getSplitData().right_plane) ++non_overlapping_inner_nodes_count;
		}
	}

	std::cout << "inner nodes: " << inner_nodes_count << " (non-overlapping: " << non_overlapping_inner_nodes_count << ")\n"
		<< "leaves: " << leaves_count << " max children count: " << max_leaf_children_count << "\n"
		<< "reserved storage: " << nodes.capacity() << " actual storage: " << nodes.size() << "\n";
}
