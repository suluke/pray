#ifndef PRAY_CUDA_PATHTRACER_TRACER_H
#define PRAY_CUDA_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "ray.hpp"
#include "bih.hpp"

#include "cuda_renderer.hpp"

template<class accel_t, class accel_cuda_t>
struct CudaPathTracer {
	using scene_t = typename accel_t::scene_t;
	using accel_pod_t = typename accel_t::pod_t;
	using material_t = EmissionMaterial;
	
	const CudaRenderer<accel_pod_t, accel_cuda_t> renderer;    // struct with device pointers
	CudaRenderer<accel_pod_t, accel_cuda_t>* d_renderer; // device pointer to struct
	
  CudaPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &accel);
	~CudaPathTracer();
	
	void render(ImageView &image);
};


#endif /* PRAY_CUDA_PATHTRACER_TRACER_H */
