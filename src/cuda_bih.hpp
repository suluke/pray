#ifndef PRAY_CUDA_BIH_H
#define PRAY_CUDA_BIH_H
#pragma once

#include "cuda_lib.hpp"
#include "scene.hpp"
#include "bih.hpp"

struct CudaBih
{
	using accel_t = BihPOD<PathScene>;
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
};

#endif
