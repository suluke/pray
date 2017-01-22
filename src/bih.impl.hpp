#include "pray/Config.h"

template<class bih_t>
struct BihBuilder
{
	typename bih_t::scene_t &scene;
	bih_t &bih;

	struct TriangleData
	{
		AABox3 aabb;
		Vector3 centroid;

		TriangleData(const AABox3 &aabb, const Vector3 &centroid) : aabb(aabb), centroid(centroid) {}
	};
	std::vector<TriangleData> triangle_data; // each element corresponds to an element in scene.triangles

	std::vector<TriangleIndex> triangles;
	typedef std::vector<TriangleIndex>::iterator TrianglesIt;

	BihBuilder(typename bih_t::scene_t &scene, bih_t &bih) : scene(scene), bih(bih) {}

	void build()
	{
		bih.pod.scene_aabb.clear();
		triangle_data.reserve(scene.triangles.size());
		for(const auto &t : scene.triangles)
		{
			auto aabb = t.calculateAabb();
			bih.pod.scene_aabb.insert(aabb);
			triangle_data.emplace_back(aabb, t.calculateCentroid());
		}

		triangles.resize(scene.triangles.size());
		for(TriangleIndex i=0u; i<scene.triangles.size(); ++i) triangles[i] = i;

		// https://de.wikipedia.org/wiki/Bin%C3%A4rbaum#Abz.C3.A4hlungen
		// The bih is a binary tree with (at most) scene.triangles.size() leaves.
		bih.pod.nodes.reserve(scene.triangles.size() + scene.triangles.size() - 1);

		bih.pod.nodes.emplace_back();
#ifdef DEBUG
		bih.pod.nodes.back().parent = nullptr;
#endif
		buildNode(bih.pod.nodes.back(), bih.pod.scene_aabb, triangles.begin(), triangles.end());

		static_assert(std::is_trivial<Triangle>::value, "Triangle should be trivial");
		std::vector<Triangle> reordered_triangles(scene.triangles.size());
		for(size_t i=0u; i<triangles.size(); ++i) reordered_triangles[i] = scene.triangles[triangles[i]];
		scene.triangles = std::move(reordered_triangles);
	}

	void buildNode(typename bih_t::pod_t::Node &current_node, const AABox3 &initial_aabb, const TrianglesIt triangles_begin, const TrianglesIt triangles_end, const unsigned retry_depth = 0u)
	{
		ASSERT(initial_aabb.isValid());

#ifdef DEBUG
		// for conditional breakpoints
		auto node_index = &current_node - &bih.pod.nodes[0];
#endif

		const auto children_count = std::distance(triangles_begin, triangles_end);

		const bool leaf_children_count_reached = children_count <= 4u; //TODO: how to pass this constant as a parameter? (pls not template...)

		/* retry_depth counts how often we retried to split a node with a smaller aabb. If is is too great (magic number), the triangles are very close to each
		   other and a split will most likely never be found (due to float inprecision). The speedup wouldn't be to great anyways... */
		const bool maximum_retry_depth_reached = retry_depth == 16u; //TODO: tweak this parameter? or find a better criterium (floating point inaccuracy reached (nextafter(aabb.min, +1.f) == aabb.max or something like that...))?

		if(leaf_children_count_reached || maximum_retry_depth_reached)
		{
			// build a leaf

			const auto children_index = std::distance(triangles.begin(), triangles_begin);
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
			const auto children_index = bih.pod.nodes.size();
			ASSERT(bih.pod.nodes.capacity() - bih.pod.nodes.size() >= 2u); // we don't want relocation (breaks references)
			static_assert(std::is_trivial<typename bih_t::Node>::value, "Bih::Node should be trivial, otherwise the critical section is not as small as it could be");
			bih.pod.nodes.emplace_back(); bih.pod.nodes.emplace_back();

			current_node.makeSplitNode(split_axis, children_index, left_plane, right_plane);

			auto &child1 = bih.pod.nodes[children_index+0];
			auto &child2 = bih.pod.nodes[children_index+1];

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

template<class ray_t, class scene_t>
void Bih<ray_t, scene_t>::build(scene_t &scene)
{
	BihBuilder<Bih<ray_t, scene_t>> builder(scene, *this);
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

template<class ray_t, class scene_t>
typename ray_t::intersect_t Bih<ray_t, scene_t>::intersect(const scene_t &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const
{
	const auto active_mask = ray.intersectAABB(pod.scene_aabb);
	if(!ray_t::isAny(active_mask)) return ray_t::getNoIntersection();

	const Vector3 direction_sign = ray.getSubrayDirection(ray_t::subrays_count / 2).sign();

	std::array<bool, 3> direction_sign_equal;
	ray.isDirectionSignEqualForAllSubrays(direction_sign, &direction_sign_equal);

	IntersectionResult<ray_t> intersection_result;

	// keen this a VALUE (as opposed to reference)!!!
	const auto triangles_data = scene.triangles.data();

	struct StackElement
	{
		float plane;
		unsigned split_axis;
		typename ray_t::bool_t active_mask;
		const typename pod_t::Node *node;
		AABox3 aabb;

		StackElement() = default;
		StackElement(float plane, unsigned split_axis, typename ray_t::bool_t active_mask, const typename pod_t::Node &node, AABox3 aabb) : plane(plane), split_axis(split_axis), active_mask(active_mask), node(&node), aabb(aabb) {}
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
		typename ray_t::bool_t active_mask;

		Current(const typename Bih<ray_t, scene_t>::pod_t::Node &node, const AABox3 &aabb, const typename ray_t::bool_t &active_mask) : node(&node), aabb(aabb), active_mask(active_mask) {}
	};

	Current current(pod.nodes[0u], pod.scene_aabb, active_mask);

	for(;;)
	{
		if(current.node->getType() == Bih<ray_t, scene_t>::pod_t::Node::Leaf)
		{
			for(unsigned i = 0u; i < current.node->getLeafData().children_count; ++i)
			{
				const TriangleIndex triangle_index = current.node->getChildrenIndex() + i;
				const Triangle &triangle = triangles_data[triangle_index];

				typename ray_t::distance_t distance = ray_t::max_distance();
				const auto intersected = ray_t::booleanAnd(ray.intersectTriangle(triangle, &distance), current.active_mask);

				if(ray_t::isAny(intersected))
				{
					// no need to pass current.active_mask here, updateIntersection handles this correctly
					// however, should ray_t ever get wider than one register, adding the mask might improve the performance if used correctly
					ray_t::updateIntersections(&intersection_result.triangle, triangle_index, &intersection_result.distance, distance);
				}
			}
		}
		else
		{
			const auto split_axis = current.node->getType();

			const auto &left_child = pod.nodes[current.node->getChildrenIndex()+0];
			const auto &right_child = pod.nodes[current.node->getChildrenIndex()+1];

			const auto left_plane = current.node->getSplitData().left_plane;
			const auto right_plane = current.node->getSplitData().right_plane;

			auto left_aabb = current.aabb;
			left_aabb.max[split_axis] = left_plane;
			auto right_aabb = current.aabb;
			right_aabb.min[split_axis] = right_plane;

			// Ignore direction_sign_equal[split_axis] here since the code would be the same. This is handeled on pop below.
			// The code in both cases is duplicated but with swapped left/right parameters. This gave a speedup of ~800ms on my machine with the sponza scene, no sse (lukasf)!
			if(direction_sign[split_axis] >= 0.f)
			{
				node_stack.push(StackElement(right_plane, split_axis, current.active_mask, right_child, right_aabb));

				const auto intersect_left = ray_t::booleanAnd(ray.intersectAABB(left_aabb), current.active_mask);
				if(ray_t::isAny(intersect_left))
				{
					current = Current(left_child, left_aabb, intersect_left);
					continue;
				}
				else
				{
					//TODO: handle right_child here, don't push it on the stack and pop it directly afterwards down below
				}
			}
			else
			{
				node_stack.push(StackElement(left_plane, split_axis, current.active_mask, left_child, left_aabb));

				const auto intersect_right = ray_t::booleanAnd(ray.intersectAABB(right_aabb), current.active_mask);
				if(ray_t::isAny(intersect_right))
				{
					current = Current(right_child, right_aabb, intersect_right);
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
				// using here that intersection_result.distance is default-initialized with ray_t::max_distance()
				ray_t::isAny(ray_t::booleanAnd(ray.intersectAxisPlane(top.plane, top.split_axis, intersection_result.distance), top.active_mask));

			if(smart_test_result)
			{
				const auto intersect_node = ray_t::booleanAnd(ray.intersectAABB(top.aabb), top.active_mask);
				if(ray_t::isAny(intersect_node))
				{
					current = Current(*top.node, top.aabb, top.active_mask);
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
void Bih<ray_t, scene_t>::printAnalysis() const
{
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
}

#include <sstream>

template<class ray_t, class scene_t>
size_t Bih<ray_t, scene_t>::hash() const
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
			s << n.getSplitData().left_plane << " " << n.getSplitData().right_plane << " ";
			f(pod.nodes[n.getChildrenIndex()+0]);
			f(pod.nodes[n.getChildrenIndex()+1]);
		}
	};
	f(pod.nodes.front());

	std::hash<std::string> hash_fn;
	return hash_fn(s.str());
}
