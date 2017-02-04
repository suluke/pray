#include "pray/Config.h"

#include <list>

template<class kdtree_t>
struct KdTreeBuilder
{
	static constexpr float cost_intersect_triangle = 1.f;
	static constexpr float cost_traversal_step = 8.f;

	typename kdtree_t::scene_t &scene;
	kdtree_t &kdtree;
	ThreadPool &thread_pool;

	std::mutex nodes_mutex;
	std::list<typename kdtree_t::Node> nodes;

	struct TriangleData
	{
		AABox3 aabb;
		Vector3 centroid;

		TriangleData(const AABox3 &aabb, const Vector3 &centroid) : aabb(aabb), centroid(centroid) {}
	};
	std::vector<TriangleData> triangle_data; // each element corresponds to an element in scene.triangles

	std::mutex out_triangles_mutex;
	std::vector<TriangleIndex> out_triangles;

	typedef std::list<TriangleIndex>::const_iterator TrianglesIt;

	KdTreeBuilder(typename kdtree_t::scene_t &scene, kdtree_t &kdtree, ThreadPool &thread_pool) : scene(scene), kdtree(kdtree), thread_pool(thread_pool) {}

	struct BuildNodeArgs
	{
		KdTreeBuilder<kdtree_t> *builder;
		typename kdtree_t::Node *current_node;
		AABox3 initial_aabb;
		std::list<TriangleIndex> triangles;
		size_t non_overlap_count;

		BuildNodeArgs() = default;
		BuildNodeArgs(KdTreeBuilder<kdtree_t> *builder, typename kdtree_t::Node *current_node, AABox3 initial_aabb, std::list<TriangleIndex> &triangles, size_t non_overlap_count) : builder(builder), current_node(current_node), initial_aabb(initial_aabb), triangles(std::move(triangles)), non_overlap_count(non_overlap_count) {}
	};

	using BuildWorker = ParallelRecursion<BuildNodeArgs>;

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
		for(TriangleIndex i=0u; i<scene.triangles.size(); ++i) triangles.push_back(i);

		out_triangles.reserve(2 * scene.triangles.size()); // just so that we are not constantly reallocating in buildLeaf (wouldn't be a problem thought)

		nodes.emplace_back();
#ifdef DEBUG
		kdtree.pod.nodes.back().parent = nullptr;
#endif

		BuildWorker worker(thread_pool);
		worker.run(buildNode, BuildNodeArgs(this, &nodes.back(), kdtree.pod.scene_aabb, triangles, triangles.size()));

		/*
		Creating the nodes directly in a std::vector is impossible since we can't estimate the maximum memory usage beforehand and reallocation breaks
		all references (especially in other worker threads!). Alternatively, we would have to add locks to all accesses to nodes (and use indices
		instead of references). Using std::list requires only the single critical section and is actually not much slower in practice.
		*/
		static_assert(std::is_trivial<typename kdtree_t::Node>::value, "Node should be trivial");
		kdtree.pod.nodes.reserve(nodes.size());
		std::copy(nodes.cbegin(), nodes.cend(), std::back_inserter(kdtree.pod.nodes));

		static_assert(std::is_trivial<Triangle>::value, "Triangle should be trivial");
		std::vector<Triangle> reordered_triangles(out_triangles.size());
		for(size_t i=0u; i<out_triangles.size(); ++i) reordered_triangles[i] = scene.triangles[out_triangles[i]];
		scene.triangles = std::move(reordered_triangles);
	}

	static float cost(const float p_l, const float p_r, const size_t n_l, const size_t n_r)
	{
		return (n_l == 0 || n_r == 0 ? .8f : 1.f) * (cost_traversal_step + cost_intersect_triangle * (p_l * n_l + p_r * n_r));
	}

	static float SAH(const AABox3 &v, const std::tuple<float, unsigned> split, const size_t n_l, const size_t n_r, const size_t n_p)
	{
		auto v_l = v, v_r = v;
		v_l.max[std::get<unsigned>(split)] = std::get<float>(split);
		v_r.min[std::get<unsigned>(split)] = std::get<float>(split);

		const auto p_l = v_l.calculateSurfaceArea() / v.calculateSurfaceArea(), p_r = v_r.calculateSurfaceArea() / v.calculateSurfaceArea();

		return std::min(cost(p_l, p_r, n_l + n_p, n_r), cost(p_l, p_r, n_l, n_r + n_p));
	}

	static float findSplitPlane(KdTreeBuilder<kdtree_t> *builder, unsigned *out_split_axis, float *out_cost, const AABox3 &aabb, const std::list<TriangleIndex> &triangles)
	{
		auto min_cost = std::numeric_limits<float>::max();
		auto min_plane = std::make_tuple(std::numeric_limits<float>::signaling_NaN(), -1u);

		const auto triangles_count = triangles.size();

		for(auto k = 0u; k < 3u; ++k)
		{
			struct Event
			{
				// don't reoder enum values, their oder is important in operator<
				enum Type
				{
					End,
					Planar,
					Start,
				};

				float plane;
				Type type;

				Event(float plane, Type type) : plane(plane), type(type) {}

				bool operator<(const Event &o) const
				{
					return plane < o.plane || (plane == o.plane && type < o.type);
				}
			};

			std::vector<Event> events;
			events.reserve(2 * triangles_count);

			for(auto t : triangles)
			{
				const auto &data = builder->triangle_data[t];
				if(data.aabb.min[k] == data.aabb.max[k])
				{
					//if(data.aabb.min[k] >= aabb.min[k] && data.aabb.max[k] <= aabb.max[k])
					{
						events.emplace_back(data.aabb.min[k], Event::Planar);
					}
				}
				else
				{
					events.emplace_back(std::max(data.aabb.min[k], aabb.min[k]), Event::Start);
					events.emplace_back(std::min(data.aabb.max[k], aabb.max[k]), Event::End);
				}
			}

			std::sort(events.begin(), events.end());

			size_t p_start = 0u, p_end = 0u;

			for(auto it = events.begin(); it != events.end(); /*noop*/)
			{
				const auto plane = it->plane;
				const auto split = std::make_tuple(plane, k);

				size_t p_planar = 0u;

				while(it != events.end() && it->plane == plane && it->type == Event::End)    { ++p_end;    ++it; }
				while(it != events.end() && it->plane == plane && it->type == Event::Planar) { ++p_planar; ++it; }
				while(it != events.end() && it->plane == plane && it->type == Event::Start)  { ++p_start;  ++it; }

				const size_t n_p = p_start - p_end;
				const size_t n_l = p_start - n_p;
				const size_t n_r = triangles_count - n_l - n_p;

				const auto c = SAH(aabb, split, n_l, n_r, n_p + p_planar);
				if(c < min_cost)
				{
					min_cost = c;
					min_plane = split;
				}

				p_start += p_planar;
				p_end += p_planar;
			}
		}

		*out_split_axis = std::get<unsigned>(min_plane);
		*out_cost = min_cost;
		return std::get<float>(min_plane);
	}

	static void buildLeaf(KdTreeBuilder<kdtree_t> *builder, typename kdtree_t::pod_t::Node &current_node, const std::list<TriangleIndex> &triangles, const size_t non_overlap_count)
	{
		const auto children_count = triangles.size();
		size_t children_index;

		{
			std::lock_guard<std::mutex> lock(builder->out_triangles_mutex);

			children_index = builder->out_triangles.size();
			static_assert(std::is_trivial<TriangleIndex>::value, "TriangleIndex should be trivial, otherwise resize has linear complexity");
			builder->out_triangles.resize(builder->out_triangles.size() + children_count);
		}

		auto it = triangles.begin();
		for(auto i=0u; i<children_count; ++i, ++it) builder->out_triangles[children_index + i] = *it;

		current_node.makeLeafNode(children_index, non_overlap_count, children_count);

#ifdef DEBUG
		//current_node.index = &current_node - &kdtree.pod.nodes[0];
		//current_node.child1 = current_node.child2 = nullptr;
#endif
	}

	static void buildNode(const BuildNodeArgs &args, BuildWorker &worker)
	{
		ASSERT(args.initial_aabb.isValid());

#ifdef DEBUG
		// for conditional breakpoints
		//const auto node_index = args.current_node - &kdtree.pod.nodes[0];
#endif

		unsigned split_axis;
		float cost_split;
		const auto split_plane = findSplitPlane(args.builder, &split_axis, &cost_split, args.initial_aabb, args.triangles);

		const auto cost_leaf = cost_intersect_triangle * args.triangles.size();

		if(cost_split > cost_leaf)
		{
			// build a leaf

			buildLeaf(args.builder, *args.current_node, args.triangles, args.non_overlap_count);
		}
		else
		{
			// build a node

			std::list<TriangleIndex> left, right;
			size_t left_non_overlap_count = 0u, right_non_overlap_count = 0u;

			for(auto t : args.triangles)
			{
				const auto &data = args.builder->triangle_data[t];

				const bool is_overlapping = (data.aabb.min[split_axis] < split_plane && data.aabb.max[split_axis] > split_plane) ||
					(data.aabb.min[split_axis] == split_plane && data.aabb.max[split_axis] == split_plane);

				if(is_overlapping)
				{
					left.push_back(t);
					right.push_back(t);
				}
				else
				{
					//TODO: we don't need the centroid here, use any triangle vertex or aabb coordinate
					const bool is_left = data.centroid[split_axis] < split_plane;
					if(is_left)
					{
						left.push_front(t);
						++left_non_overlap_count;
					}
					else
					{
						right.push_front(t);
						++right_non_overlap_count;
					}
				}
			}

			if(left_non_overlap_count == 0u)
			{
				buildLeaf(args.builder, *args.current_node, right, right_non_overlap_count);
				return;
			}
			if(right_non_overlap_count == 0u)
			{
				buildLeaf(args.builder, *args.current_node, left, left_non_overlap_count);
				return;
			}

			typename kdtree_t::Node *child1, *child2;

			size_t children_index;
			{
				std::lock_guard<std::mutex> lock(args.builder->nodes_mutex);

				children_index = args.builder->nodes.size();
				static_assert(std::is_trivial<typename kdtree_t::Node>::value, "KdTree::Node should be trivial, otherwise the critical section is not as small as it could be");
				child1 = &args.builder->nodes.emplace_back();
				child2 = &args.builder->nodes.emplace_back();
			}

			args.current_node->makeSplitNode(split_axis, children_index, split_plane);

#ifdef DEBUG
			//args.current_node.index = node_index;
			//args.current_node.child1 = &child1;
			//args.current_node.child2 = &child2;
			//child1.parent = child2.parent = &current_node;
#endif

			AABox3 child1_initial_aabb = args.initial_aabb, child2_initial_aabb = args.initial_aabb;
			child1_initial_aabb.max[split_axis] = split_plane;
			child2_initial_aabb.min[split_axis] = split_plane;

			// recursion
			worker.recurse(buildNode, BuildNodeArgs(args.builder, child1, child1_initial_aabb, left, left_non_overlap_count));
			buildNode(BuildNodeArgs(args.builder, child2, child2_initial_aabb, right, right_non_overlap_count), worker);
		}
	}
};

template<class ray_t, class scene_t>
void KdTree<ray_t, scene_t>::build(scene_t &scene, ThreadPool &thread_pool)
{
	KdTreeBuilder<KdTree<ray_t, scene_t>> builder(scene, *this, thread_pool);
	builder.build();
}

#ifdef DEBUG_TOOL
extern std::vector<size_t> kdtree_intersected_nodes;
#endif

template<class ray_t, class scene_t>
typename ray_t::intersect_t KdTree<ray_t, scene_t>::intersect(const scene_t &scene, const ray_t &ray, const typename ray_t::bool_t ray_active_mask, typename ray_t::distance_t *out_distance) const
{
#ifdef DEBUG_TOOL
	kdtree_intersected_nodes.clear();
#endif

	auto active_mask = ray_t::booleanAnd(ray.intersectAABB(pod.scene_aabb), ray_active_mask);
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
		kdtree_intersected_nodes.push_back(current.node - &pod.nodes[0]);
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

					//TODO: what to do about that?
					if(direction_sign_equal[0] && direction_sign_equal[1] && direction_sign_equal[2] && i < current.node->getLeafData().non_overlap_count)
					{
						active_mask = ray_t::booleanAnd(ray_t::booleanNot(intersected), active_mask);
					}
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
	std::cout << "nodes: " << pod.nodes.size() << "\n";
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
