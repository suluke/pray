#ifndef PRAY_CUDA_KDTREE_H
#define PRAY_CUDA_KDTREE_H
#pragma once

#include "cuda_lib.hpp"
#include "cuda_scene.hpp"
#include "cuda_ray.hpp"
#include "scene.hpp"
#include "kdtree.hpp"

#include <cuda_runtime_api.h>

struct CudaKdTree
{
	using accel_t = KdTreePOD<PathScene>;
	using pod_t = accel_t;
	using scene_t = CudaScene;
	using ray_t = CudaRay;
	using Node = typename KdTreePOD<PathScene>::Node;
	
	struct StackElement
	{
		float plane;
		unsigned split_axis;
		const typename pod_t::Node *node;
		AABox3 aabb;

		__device__ StackElement() = default;
		__device__ StackElement(float plane, unsigned split_axis, const typename pod_t::Node &node, AABox3 aabb) : plane(plane), split_axis(split_axis), node(&node), aabb(aabb) {}
	};

	struct Stack
	{
		static const size_t node_stack_size = 128u;
		StackElement node_stack[node_stack_size];
		int node_stack_pointer = -1;

		__device__ bool empty()
		{
			return node_stack_pointer == -1;
		}

		__device__ bool full()
		{
			return node_stack_pointer == (int) node_stack_size - 1;
		}

		// Can't use template<class... Args> here because templates are not allowed inside functions.
		// http://stackoverflow.com/questions/3449112/why-cant-templates-be-declared-in-a-function
		__device__ void push(const StackElement &element)
		{
			ASSERT(!full());
			node_stack[++node_stack_pointer] = element;
		}

		__device__ StackElement &pop()
		{
			ASSERT(!empty());
			return node_stack[node_stack_pointer--];
		}
	};
	
	struct Current
	{
		const typename pod_t::Node *node;
		AABox3 aabb;

		__device__ Current(const Node &node, const AABox3 &aabb) : node(&node), aabb(aabb) {}
	};
	
	cuda::vector<Node> nodes;
	AABox3* scene_aabb;
	
	// for calculation of stack size need
	static const size_t stack_size = sizeof(StackElement) * Stack::node_stack_size;
	
	CudaKdTree(const CudaKdTree&);
	CudaKdTree(const CudaKdTree&&);
	
	CudaKdTree(const accel_t &pod)
	{
		// allocate memory and copy data
		nodes = cuda::vector<Node>::create(pod.nodes);
		scene_aabb = cuda::create<AABox3>(pod.scene_aabb);
	}
	
	~CudaKdTree()
	{
		// free memory
		nodes.destroy();
		cuda::destroy<AABox3>(scene_aabb);
	}
	
	
	__device__ ray_t::intersect_t intersect(const scene_t &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const;
};

#endif
