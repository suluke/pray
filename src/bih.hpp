#ifndef PRAY_BIH_H
#define PRAY_BIH_H

#include "scene.hpp"

// http://ainc.de/Research/BIH.pdf

template<class ray_t>
struct Bih
{
	//TODO: compactify this (merge children_index and type)
	struct Node
	{
		enum Type
		{
			// Split_* must be 0, 1, 2 ! (see static_cast in BihBuilder::buildNode)
			Split_X = 0,
			Split_Y = 1,
			Split_Z = 2,

			Leaf = 3,
		} type;

		union Data
		{
			struct SplitData
			{
				float left_plane, right_plane;
				uint32_t children_index;
			} split;

			struct LeafData
			{
				uint32_t children_index;
				uint32_t children_count;
			} leaf;
		} data;

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
	TriangleIndex intersect(const Scene &scene, const ray_t &ray, float *out_distance) const;
};

#include "bih.impl.hpp"

#endif
