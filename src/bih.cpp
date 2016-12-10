#include "bih.hpp"

struct BihBuilder
{
	const Scene &scene;
	Bih &bih;

	struct TriangleData
	{
		AABox3 aabb;
		Vector3 centroid;

		TriangleData(const AABox3 &aabb, const Vector3 &centroid) : aabb(aabb), centroid(centroid) {}
	};
	std::vector<TriangleData> triangle_data; // each element corresponds to an element in scene.triangles

	typedef std::vector<TriangleIndex>::iterator TrianglesIt;

	BihBuilder(const Scene &scene, Bih &bih) : scene(scene), bih(bih) {}

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

		//TODO: magic number !!!!
		bih.nodes.reserve(10000u);

		bih.nodes.emplace_back();
#ifdef DEBUG
		bih.nodes.back().parent = nullptr;
#endif
		buildNode(bih.nodes.back(), bih.scene_aabb, bih.triangles.begin(), bih.triangles.end());
	}

	void buildNode(Bih::Node &current_node, const AABox3 &initial_aabb, TrianglesIt triangles_begin, TrianglesIt triangles_end)
	{
		ASSERT(initial_aabb.isValid());

#ifdef DEBUG
		// for conditional breakpoints
		auto node_index = &current_node - &bih.nodes[0];
#endif

		auto children_count = std::distance(triangles_begin, triangles_end);

		//TODO: if two triangles happen to have the same centroid, this recursion never ends...

		if(children_count != 1)
		{
			// build a node

			auto split_axis = getSplitAxis(initial_aabb);

			current_node.type = static_cast<Bih::Node::Type>(split_axis);

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

			current_node.data.split.left_plane = left_plane;
			current_node.data.split.right_plane = right_plane;

			// allocate child nodes (this is a critical section)
			current_node.data.split.children_index = bih.nodes.size();
			ASSERT(bih.nodes.capacity() - bih.nodes.size() >= 2u); // we don't want relocation (breaks iterators and references)
			bih.nodes.emplace_back(); bih.nodes.emplace_back();
			auto &child1 = bih.nodes[current_node.data.split.children_index+0];
			auto &child2 = bih.nodes[current_node.data.split.children_index+1];

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
			current_node.type = Bih::Node::Leaf;
			current_node.data.leaf.children_index = std::distance(bih.triangles.begin(), triangles_begin);
			current_node.data.leaf.children_count = children_count;

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

void Bih::build(const Scene &scene)
{
	BihBuilder builder(scene, *this);
	builder.build();
}

TriangleIndex Bih::intersectTriangle(const Scene &scene, const Ray &ray, float *out_distance) const
{
	//TODO
	ASSERT(false);
	return TriangleIndex_Invalid;
}
