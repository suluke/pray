#ifndef PRAY_BIH_H
#define PRAY_BIH_H

#include "scene.hpp"

// http://ainc.de/Research/BIH.pdf

template<class ray_t>
struct Bih
{
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

		Type getType() const
		{
			return static_cast<Type>(type_and_children_index >> 30);
		}

		uint32_t getChildrenIndex() const
		{
			return type_and_children_index & ~(3 << 30);
		}

		const SplitData &getSplitData() const
		{
			ASSERT(getType() != Leaf);
			return data.split;
		}

		const LeafData &getLeafData() const
		{
			ASSERT(getType() == Leaf);
			return data.leaf;
		}

		private:
		uint32_t type_and_children_index;

		void setTypeAndChildrenIndex(Type type, uint32_t children_index)
		{
			ASSERT(type >= 0 && type <= 3);
			ASSERT(children_index < (2u << 30));
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

	std::vector<TriangleIndex> triangles;

	AABox3 scene_aabb;
	std::vector<Node> nodes;

	void build(const Scene &scene);
	typename ray_t::intersect_t intersect(const Scene &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const;

	void printAnalysis() const;
};

#include "bih.impl.hpp"

#endif
