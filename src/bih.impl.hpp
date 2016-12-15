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

	void buildNode(typename bih_t::Node &current_node, const AABox3 &initial_aabb, TrianglesIt triangles_begin, TrianglesIt triangles_end)
	{
		ASSERT(initial_aabb.isValid());

#ifdef DEBUG
		// for conditional breakpoints
		auto node_index = &current_node - &bih.nodes[0];
#endif

		auto children_count = std::distance(triangles_begin, triangles_end);

		//TODO: if two triangles happen to have the same centroid, this recursion never ends...

		//TODO: how to pass this constant as a parameter? (pls not template...)
		if(children_count > 4)
		{
			// build a node

			auto split_axis = getSplitAxis(initial_aabb);

			float left_plane = std::numeric_limits<float>::lowest(), right_plane = std::numeric_limits<float>::max();

			auto pivot = (initial_aabb.min[split_axis] + initial_aabb.max[split_axis]) / 2.f;
			auto split_element = std::partition(triangles_begin, triangles_end,
			[&](TriangleIndex t)
			{
				const bool left = triangle_data[t].centroid[split_axis] <= pivot;

				if(left) left_plane = std::max(left_plane, triangle_data[t].aabb.max[split_axis]);
				else right_plane = std::min(right_plane, triangle_data[t].aabb.min[split_axis]);

				return left;
			});

			if(split_element == triangles_begin || split_element == triangles_end)
			{
				AABox3 new_initial_aabb = initial_aabb;
				if(split_element == triangles_begin) new_initial_aabb.min[split_axis] = pivot;
				else                                 new_initial_aabb.max[split_axis] = pivot;

				return buildNode(current_node, new_initial_aabb, triangles_begin, triangles_end);
			}

			// allocate child nodes (this is a critical section)
			auto children_index = bih.nodes.size();
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

			buildNode(child1, child1_initial_aabb, triangles_begin, split_element);
			buildNode(child2, child2_initial_aabb, split_element, triangles_end);
		}
		else
		{
			// build a leaf

			auto children_index = std::distance(bih.triangles.begin(), triangles_begin);
			current_node.makeLeafNode(children_index, children_count);

#ifdef DEBUG
			current_node.child1 = current_node.child2 = nullptr;
#endif
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
struct IntersectionResult
{
	TriangleIndex triangle = TriangleIndex_Invalid;
	float distance = std::numeric_limits<float>::max();
};

template<class ray_t>
static void intersectBihNode(const typename Bih<ray_t>::Node &node, const AABox3 aabb, const Bih<ray_t> &bih, const Scene &scene, const ray_t &ray, IntersectionResult &intersection)
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

			float distance;
			if(ray.intersectTriangle(triangle, &distance))
			{
				if(distance < intersection.distance)
				{
					intersection.triangle = triangle_index;
					intersection.distance = distance;
				}
			}
		}
	}
	else
	{
		auto split_axis = node.getType();

		auto &left_child = bih.nodes[node.getChildrenIndex()+0];
		auto &right_child = bih.nodes[node.getChildrenIndex()+1];

		auto left_plane = node.getSplitData().left_plane;
		auto right_plane = node.getSplitData().right_plane;

		AABox3 left_aabb = aabb, right_aabb = aabb;
		left_aabb.max[split_axis] = left_plane;
		right_aabb.min[split_axis] = right_plane;

		//TODO: this can be done smarter!
		auto intersect_left = intersectRayAABB(ray.origin, ray.direction, left_aabb);
		if(intersect_left) intersectBihNode(left_child, left_aabb, bih, scene, ray, intersection);
		auto intersect_right = intersectRayAABB(ray.origin, ray.direction, right_aabb);
		if(intersect_right) intersectBihNode(right_child, right_aabb, bih, scene, ray, intersection);
	}
}

template<class ray_t>
TriangleIndex Bih<ray_t>::intersect(const Scene &scene, const ray_t &ray, float *out_distance) const
{
#ifdef DEBUG_TOOL
	bih_intersected_nodes.clear();
#endif

	if(!intersectRayAABB(ray.origin, ray.direction, scene_aabb)) return TriangleIndex_Invalid;

	IntersectionResult intersection_result;
	intersectBihNode(nodes[0u], scene_aabb, *this, scene, ray, intersection_result);

	*out_distance = intersection_result.distance;
	return intersection_result.triangle;
}
