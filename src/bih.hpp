#ifndef PRAY_BIH_H
#define PRAY_BIH_H

#include "scene.hpp"
#include "cuda.hpp"
#include "parallel_worker.hpp"

#ifdef WITH_CUDA
	struct CudaBih;
#endif

// http://ainc.de/Research/BIH.pdf

template<class SCENE_T>
struct BihPOD
{
	using scene_t = SCENE_T;
	struct Node
	{
		enum Type
		{
			// Split_* must be 0, 1, 2 ! (see static_cast in makeSplitNode)
			Split_X = 0,
			Split_Y = 1,
			Split_Z = 2,

			Leaf = 3,
		};

		struct SplitData
		{
			float left_plane, right_plane;
		};

		struct LeafData
		{
			uint32_t children_count;
		};

		void makeSplitNode(unsigned split_axis, uint32_t children_index, float left_plane, float right_plane)
		{
			setTypeAndChildrenIndex(static_cast<Type>(split_axis), children_index);
			data.split.left_plane = left_plane;
			data.split.right_plane = right_plane;
		}

		void makeLeafNode(uint32_t children_index, uint32_t children_count)
		{
			setTypeAndChildrenIndex(Leaf, children_index);
			data.leaf.children_count = children_count;
		}

		__cuda__ Type getType() const
		{
			return static_cast<Type>(type_and_children_index >> 30);
		}

		__cuda__ uint32_t getChildrenIndex() const
		{
			return type_and_children_index & ~(3 << 30);
		}

		__cuda__ const SplitData &getSplitData() const
		{
			ASSERT(getType() != Leaf);
			return data.split;
		}

		__cuda__ const LeafData &getLeafData() const
		{
			ASSERT(getType() == Leaf);
			return data.leaf;
		}

		private:
		uint32_t type_and_children_index;

		void setTypeAndChildrenIndex(Type type, uint32_t children_index)
		{
			ASSERT(type >= 0 && type <= 3);
			ASSERT(children_index < (1u << 30));
			type_and_children_index = type << 30 | children_index;
		}

		union Data
		{
			SplitData split;
			LeafData leaf;
		} data;

		public:
#ifdef DEBUG
		// for debugging
		Node *parent, *child1, *child2;
		float split_point;
		unsigned index;
#endif
	};

	AABox3 scene_aabb;
	std::vector<Node> nodes;
};

template<class ray_t, class SCENE_T>
struct Bih
{
	using scene_t = SCENE_T;
	using pod_t = BihPOD<scene_t>;
	using Node = typename pod_t::Node;
	
	#ifdef WITH_CUDA
		using accel_cuda_t = CudaBih;
	#endif
	
	pod_t pod; // created by auto default constructor
	
	void build(scene_t &scene, ThreadPool &thread_pool);
	typename ray_t::intersect_t intersect(const scene_t &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const;

	void printAnalysis() const;
	size_t hash() const;
};

#include "bih.impl.hpp"

#endif
