#ifndef PRAY_CUDA_BIH_H
#define PRAY_CUDA_BIH_H
#pragma once

#include "cuda.hpp"
#include "bih.hpp"

template<class scene_t>
struct CudaBih
{
	using pod_t = BihPOD<scene_t>;
	using Node = typename pod_t::Node;
	
	pod_t &bih;
	
	using accel_pod_t = pod_t;
	
	cuda::vector<Node> nodes;
	AABox3* scene_aabb;
	
	CudaBih(pod_t &bih) : bih(bih) {}
	
	void initialize()
	{
		// allocate memory and copy data
		nodes = cuda::vector<Node>::create(bih.nodes);
		scene_aabb = cuda::create<AABox3>(bih.scene_aabb);
	}
	
	void finalize()
	{
		// free memory
		nodes.destroy();
		cuda::destroy<AABox3>(scene_aabb);
	}
};

#endif
