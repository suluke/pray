#ifndef PRAY_CUDA_BIH_H
#define PRAY_CUDA_BIH_H
#pragma once

#include "cuda_lib.hpp"
#include "cuda_scene.hpp"
#include "cuda_ray.hpp"
#include "scene.hpp"
#include "bih.hpp"

struct CudaBih
{
	using accel_t = BihPOD<PathScene>;
	using scene_t = CudaScene;
	using ray_t = CudaRay;
	using Node = typename accel_t::Node;
	
	cuda::vector<Node> nodes;
	AABox3* scene_aabb;
	
	CudaBih(const CudaBih&);
	CudaBih(const CudaBih&&);
	
	CudaBih(const accel_t &pod)
	{
		// allocate memory and copy data
		nodes = cuda::vector<Node>::create(pod.nodes);
		scene_aabb = cuda::create<AABox3>(pod.scene_aabb);
	}
	
	~CudaBih()
	{
		// free memory
		nodes.destroy();
		cuda::destroy<AABox3>(scene_aabb);
	}
	
	
	__device__ ray_t::intersect_t intersect(const scene_t &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const;
};

#endif
