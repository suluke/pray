#include "pray/Config.h"

#include <list>

template<class kdtree_t>
struct KdTreeBuilder
{
	static constexpr float cost_intersect_triangle = 1.f;
	static constexpr float cost_traversal_step = 1.f;

	typename kdtree_t::scene_t &scene;
	kdtree_t &kdtree;

	struct TriangleData
	{
		AABox3 aabb;
		Vector3 centroid;

		TriangleData(const AABox3 &aabb, const Vector3 &centroid) : aabb(aabb), centroid(centroid) {}
	};
	std::vector<TriangleData> triangle_data; // each element corresponds to an element in scene.triangles

	std::vector<TriangleIndex> out_triangles;

	typedef std::list<TriangleIndex>::const_iterator TrianglesIt;

	KdTreeBuilder(typename kdtree_t::scene_t &scene, kdtree_t &kdtree) : scene(scene), kdtree(kdtree) {}

	void build()
	{
		kdtree.pod.scene_aabb.clear();
		triangle_data.reserve(scene.triangles.size());
		for(const auto &t : scene.triangles)
		{
			auto aabb = t.calculateAabb();
			kdtree.pod.scene_aabb.insert(aabb);
			triangle_data.emplace_back(aabb, t.calculateCentroid());
		}

		std::list<TriangleIndex> triangles;
		//triangles.reserve(scene.triangles.size() * scene.triangles.size());
		for(TriangleIndex i=0u; i<scene.triangles.size(); ++i) triangles.push_back(i);

		kdtree.pod.nodes.reserve(2 * (scene.triangles.size() + scene.triangles.size() - 1)); //TODO: find a good number here, we MUST NOT reallocate in buildNode!!!

		kdtree.pod.nodes.emplace_back();
#ifdef DEBUG
		kdtree.pod.nodes.back().parent = nullptr;
#endif
		buildNode(kdtree.pod.nodes.back(), kdtree.pod.scene_aabb, triangles.cbegin(), triangles.cend());

		static_assert(std::is_trivial<Triangle>::value, "Triangle should be trivial");
		std::vector<Triangle> reordered_triangles(out_triangles.size());
		for(size_t i=0u; i<out_triangles.size(); ++i) reordered_triangles[i] = scene.triangles[out_triangles[i]];
		scene.triangles = std::move(reordered_triangles);
	}

	std::tuple<float, unsigned> getPerfectSplit(const TriangleIndex t, const AABox3 &aabb, const unsigned i)
	{
		const auto &data = triangle_data[t];

		switch(i)
		{
			case 0u: return std::make_tuple(std::max(data.aabb.min[0u], aabb.min[0u]), 0u);
			case 1u: return std::make_tuple(std::min(data.aabb.max[0u], aabb.max[0u]), 0u);
			case 2u: return std::make_tuple(std::max(data.aabb.min[1u], aabb.min[1u]), 1u);
			case 3u: return std::make_tuple(std::min(data.aabb.max[1u], aabb.max[1u]), 1u);
			case 4u: return std::make_tuple(std::max(data.aabb.min[2u], aabb.min[2u]), 2u);
			case 5u: return std::make_tuple(std::min(data.aabb.max[2u], aabb.max[2u]), 2u);
			default: return std::make_tuple(std::numeric_limits<float>::signaling_NaN(), -1u);
		}
	}

	float cost(const float p_l, const float p_r, const size_t n_l, const size_t n_r)
	{
		return /*l(p) * */(cost_traversal_step + cost_intersect_triangle * (p_l * n_l + p_r * n_r));
	}

	float SAH(const AABox3 &v, const std::tuple<float, unsigned> split, const size_t n_l, const size_t n_r, const size_t n_p)
	{
		auto v_l = v, v_r = v;
		v_l.max[std::get<unsigned>(split)] = std::get<float>(split);
		v_r.min[std::get<unsigned>(split)] = std::get<float>(split);

		const auto p_l = v_l.calculateSurfaceArea() / v.calculateSurfaceArea(), p_r = v_r.calculateSurfaceArea() / v.calculateSurfaceArea();

		return std::min(cost(p_l, p_r, n_l + n_p, n_r), cost(p_l, p_r, n_l, n_r + n_p));
	}

	float findSplitPlane(unsigned *out_split_axis, float *out_cost, size_t *out_triangles_count, const AABox3 &aabb, const TrianglesIt triangles_begin, const TrianglesIt triangles_end)
	{
		auto min_cost = std::numeric_limits<float>::max();
		auto min_plane = std::make_tuple(std::numeric_limits<float>::signaling_NaN(), -1u);

		size_t count = 0u;

		for(auto it = triangles_begin; it != triangles_end; ++it)
		{
			++count;

			for(auto i = 0u; i < 6u; ++i)
			{
				const auto split = getPerfectSplit(*it, aabb, i);

				size_t l = 0u, r = 0u, p = 0u;

				for(auto it = triangles_begin; it != triangles_end; ++it)
				{
					const auto &data = triangle_data[*it];
					const bool overlapping = data.aabb.min[std::get<unsigned>(split)] < std::get<float>(split) && data.aabb.max[std::get<unsigned>(split)] > std::get<float>(split);
					if(overlapping) ++p;
					else
					{
						if(data.centroid[std::get<unsigned>(split)] < std::get<float>(split)) ++l;
						else ++r;
					}
				}

				const auto c = SAH(aabb, split, l, r, p);

				if(c < min_cost)
				{
					min_cost = c;
					min_plane = split;
				}
			}
		}

		*out_split_axis = std::get<unsigned>(min_plane);
		*out_cost = min_cost;
		*out_triangles_count = count; // just an optimization so that we don't have to iterate through the triangles list multiple times
		return std::get<float>(min_plane);
	}

	void buildLeaf(typename kdtree_t::pod_t::Node &current_node, const TrianglesIt triangles_begin, const TrianglesIt triangles_end, const size_t triangles_count)
	{
		const auto children_index = out_triangles.size();
		const auto children_count = triangles_count;
		for(auto it = triangles_begin; it != triangles_end; ++it) out_triangles.push_back(*it);
		current_node.makeLeafNode(children_index, children_count);

#ifdef DEBUG
		current_node.child1 = current_node.child2 = nullptr;
#endif
	}

	void buildNode(typename kdtree_t::pod_t::Node &current_node, const AABox3 &initial_aabb, const TrianglesIt triangles_begin, const TrianglesIt triangles_end)
	{
		ASSERT(initial_aabb.isValid());

#ifdef DEBUG
		// for conditional breakpoints
		const auto node_index = &current_node - &kdtree.pod.nodes[0];
#endif

		size_t triangles_count;

		unsigned split_axis;
		float cost_split;
		const auto split_plane = findSplitPlane(&split_axis, &cost_split, &triangles_count, initial_aabb, triangles_begin, triangles_end);

		const auto cost_leaf = cost_intersect_triangle * triangles_count;

		if(cost_split > cost_leaf)
		{
			// build a leaf

			buildLeaf(current_node, triangles_begin, triangles_end, triangles_count);
		}
		else
		{
			// build a node

			//TODO: yeah, this is a copy...
			std::list<TriangleIndex> node_triangles(triangles_begin, triangles_end);

			const auto o = std::partition(node_triangles.begin(), node_triangles.end(),
			[&](TriangleIndex t)
			{
				const bool overlapping = triangle_data[t].aabb.min[split_axis] < split_plane && triangle_data[t].aabb.max[split_axis] > split_plane;
				return !overlapping;
			});

			const auto overlapping_begin = o;
			const auto overlapping_end = node_triangles.end();

			const auto p = std::partition(node_triangles.begin(), overlapping_begin,
			[&](TriangleIndex t)
			{
				return triangle_data[t].centroid[split_axis] < split_plane;
			});

			const auto left_begin = node_triangles.begin();
			const auto left_end = p;
			const auto right_begin = p;
			const auto right_end = overlapping_begin;

			if(left_begin == left_end)
			{
				buildLeaf(current_node, right_begin, overlapping_end, triangles_count);
				return;
			}
			if(right_begin == right_end)
			{
				//TODO: sooo much efficiency...
				node_triangles.insert(left_end, overlapping_begin, overlapping_end);

				buildLeaf(current_node, left_begin, left_end, triangles_count);
				return;
			}

			//TODO: sooo much efficiency...
			node_triangles.insert(left_end, overlapping_begin, overlapping_end);

			const auto children_index = kdtree.pod.nodes.size();
			ASSERT(kdtree.pod.nodes.capacity() - kdtree.pod.nodes.size() >= 2u); // we don't want relocation (breaks references)
			static_assert(std::is_trivial<typename kdtree_t::Node>::value, "KdTree::Node should be trivial, otherwise the critical section is not as small as it could be");
			kdtree.pod.nodes.emplace_back(); kdtree.pod.nodes.emplace_back();

			current_node.makeSplitNode(split_axis, children_index, split_plane);

			auto &child1 = kdtree.pod.nodes[children_index+0];
			auto &child2 = kdtree.pod.nodes[children_index+1];

#ifdef DEBUG
			current_node.index = node_index;
			current_node.child1 = &child1;
			current_node.child2 = &child2;
			child1.parent = child2.parent = &current_node;
#endif

			AABox3 child1_initial_aabb = initial_aabb, child2_initial_aabb = initial_aabb;
			child1_initial_aabb.max[split_axis] = split_plane;
			child2_initial_aabb.min[split_axis] = split_plane;

			// recursion
			buildNode(child1, child1_initial_aabb, left_begin, left_end);
			buildNode(child2, child2_initial_aabb, right_begin, overlapping_end);
		}
	}
};

template<class ray_t, class scene_t>
void KdTree<ray_t, scene_t>::build(scene_t &scene)
{
	KdTreeBuilder<KdTree<ray_t, scene_t>> builder(scene, *this);
	builder.build();
}

#ifdef DEBUG_TOOL
extern std::vector<size_t> kdtree_intersected_nodes;
#endif

template<class ray_t, class scene_t>
typename ray_t::intersect_t KdTree<ray_t, scene_t>::intersect(const scene_t &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const
{
#ifdef DEBUG_TOOL
	kdtree_intersected_nodes.clear();
#endif

	auto active_mask = ray.intersectAABB(pod.scene_aabb);
	if(!ray_t::isAny(active_mask)) return ray_t::getNoIntersection();

	const Vector3 direction_sign = ray.getSubrayDirection(ray_t::subrays_count / 2).sign();

	std::array<bool, 3> direction_sign_equal;
	ray.isDirectionSignEqualForAllSubrays(direction_sign, &direction_sign_equal);

	// move to global namespace and use this intead of float *out_distance?
	struct IntersectionResult
	{
		typename ray_t::intersect_t triangle = ray_t::getNoIntersection();
		typename ray_t::distance_t distance = ray_t::max_distance();
	};

	IntersectionResult intersection_result;

	// keep this a VALUE (as opposed to reference)!!!
	const auto triangles_data = scene.triangles.data();

	struct StackElement
	{
		float plane;
		unsigned split_axis;
		const typename pod_t::Node *node;
		AABox3 aabb;

		StackElement() = default;
		StackElement(float plane, unsigned split_axis, const typename pod_t::Node &node, AABox3 aabb) : plane(plane), split_axis(split_axis), node(&node), aabb(aabb) {}
	};
	static_assert(std::is_trivial<StackElement>::value, "StackElement should be trivial, otherwise we pay for stack initialization");

	struct Stack
	{
		std::array<StackElement, 128u> node_stack;
		int node_stack_pointer = -1;

		bool empty()
		{
			return node_stack_pointer == -1;
		}

		bool full()
		{
			return node_stack_pointer == (int) node_stack.size() - 1;
		}

		// Can't use template<class... Args> here because templates are not allowed inside functions.
		// http://stackoverflow.com/questions/3449112/why-cant-templates-be-declared-in-a-function
		void push(const StackElement &element)
		{
			ASSERT(!full());
			node_stack[++node_stack_pointer] = element;
		}

		StackElement &pop()
		{
			ASSERT(!empty());
			return node_stack[node_stack_pointer--];
		}
	};

	Stack node_stack;

	struct Current
	{
		const typename pod_t::Node *node;
		AABox3 aabb;

		Current(const typename KdTree<ray_t, scene_t>::pod_t::Node &node, const AABox3 &aabb) : node(&node), aabb(aabb) {}
	};

	Current current(pod.nodes[0u], pod.scene_aabb);

	for(;;)
	{
#ifdef DEBUG_TOOL
		kdtree_intersected_nodes.push_back(current.node - &kdtree.pod.nodes[0]);
#endif

		if(current.node->getType() == KdTree<ray_t, scene_t>::pod_t::Node::Leaf)
		{
			const auto old_active_mask = active_mask;

			for(unsigned i = 0u; i < current.node->getLeafData().children_count; ++i)
			{
				const TriangleIndex triangle_index = current.node->getChildrenIndex() + i;
				const Triangle &triangle = triangles_data[triangle_index];

				typename ray_t::distance_t distance = ray_t::max_distance();
				const auto intersected = ray_t::booleanAnd(ray.intersectTriangle(triangle, &distance), old_active_mask);

				if(ray_t::isAny(intersected))
				{
					// no need to pass old_active_mask here, updateIntersection handles this correctly
					// however, should ray_t ever get wider than one register, adding the mask might improve the performance if used correctly
					ray_t::updateIntersections(&intersection_result.triangle, triangle_index, &intersection_result.distance, distance);
				}

				//TODO: what to do about that?
				if(direction_sign_equal[0] && direction_sign_equal[1] && direction_sign_equal[2])
				{
					active_mask = ray_t::booleanAnd(active_mask, intersected);
				}
			}
		}
		else
		{
			const auto split_axis = current.node->getType();

			const auto &left_child = pod.nodes[current.node->getChildrenIndex()+0];
			const auto &right_child = pod.nodes[current.node->getChildrenIndex()+1];

			const auto split_plane = current.node->getSplitData().plane;

			auto left_aabb = current.aabb;
			left_aabb.max[split_axis] = split_plane;
			auto right_aabb = current.aabb;
			right_aabb.min[split_axis] = split_plane;

			// Ignore direction_sign_equal[split_axis] here since the code would be the same. This is handeled on pop below.
			// The code in both cases is duplicated but with swapped left/right parameters. This gave a speedup of ~800ms on my machine with the sponza scene, no sse (lukasf)!
			if(direction_sign[split_axis] >= 0.f)
			{
				node_stack.push(StackElement(split_plane, split_axis, right_child, right_aabb));

				const auto intersect_left = ray_t::booleanAnd(ray.intersectAABB(left_aabb), active_mask);
				if(ray_t::isAny(intersect_left))
				{
					current = Current(left_child, left_aabb);
					continue;
				}
				else
				{
					//TODO: handle right_child here, don't push it on the stack and pop it directly afterwards down below
				}
			}
			else
			{
				node_stack.push(StackElement(split_plane, split_axis, left_child, left_aabb));

				const auto intersect_right = ray_t::booleanAnd(ray.intersectAABB(right_aabb), active_mask);
				if(ray_t::isAny(intersect_right))
				{
					current = Current(right_child, right_aabb);
					continue;
				}
				else
				{
					//TODO: handle left_child here, don't push it on the stack and pop it directly afterwards down below
				}
			}
		}

		for(;;)
		{
			if(node_stack.empty()) goto finish; // this goto is totally valid, I need to jump out of two loops!

			const auto &top = node_stack.pop(); // the reference is valid as long as nothing is pushed on the stack

			const auto smart_test_result = !direction_sign_equal[top.split_axis] || // always handle nodes for which !direction_sign_equal[split_axis]
				ray_t::isAny(active_mask);

			if(smart_test_result)
			{
				const auto intersect_node = ray_t::booleanAnd(ray.intersectAABB(top.aabb), active_mask);
				if(ray_t::isAny(intersect_node))
				{
					current = Current(*top.node, top.aabb);
					break;
				}
			}
		}
	}

	finish:

	*out_distance = intersection_result.distance;
	return intersection_result.triangle;
}

template<class ray_t, class scene_t>
void KdTree<ray_t, scene_t>::printAnalysis() const
{
	/*
	size_t inner_nodes_count = 0u, non_overlapping_inner_nodes_count = 0u, leaves_count = 0u, max_leaf_children_count = 0u;

	for(auto &n : pod.nodes)
	{
		if(n.getType() == pod_t::Node::Leaf)
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
		<< "reserved storage: " << pod.nodes.capacity() << " actual storage: " << pod.nodes.size() << "\n";
	*/
}

#include <sstream>

template<class ray_t, class scene_t>
size_t KdTree<ray_t, scene_t>::hash() const
{
	std::stringstream s;

	s << pod.scene_aabb.min << " " << pod.scene_aabb.max << " ";

	const std::function<void(const typename pod_t::Node&)> f = [&](const typename pod_t::Node &n)
	{
		s << n.getType() << " ";
		if(n.getType() == pod_t::Node::Leaf)
		{
			for(auto i=0u; i<n.getLeafData().children_count; ++i) s << n.getChildrenIndex()+i << " ";
		}
		else
		{
			s << n.getSplitData().plane << " ";
			f(pod.nodes[n.getChildrenIndex()+0]);
			f(pod.nodes[n.getChildrenIndex()+1]);
		}
	};
	f(pod.nodes.front());

	std::hash<std::string> hash_fn;
	return hash_fn(s.str());
}
