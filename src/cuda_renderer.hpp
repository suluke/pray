#ifndef PRAY_CUDA_RENDERER_H
#define PRAY_CUDA_RENDERER_H
#pragma once

#include "scene.hpp"
#include "ray.hpp"
#include "bih.hpp"

#include "cuda_image.hpp"
#include "cuda_ray.hpp"
#include "cuda_bih.hpp"
#include "cuda_scene.hpp"

#include <cuda_runtime_api.h>
#include <curand_kernel.h>

template<class accel_cuda_t>
struct CudaRenderer {
	using scene_cuda_t = CudaScene;
	using scene_t = PathScene;
	
	using ray_t = CudaRay;
	
	const scene_cuda_t scene;
	const accel_cuda_t accel;
	RenderOptions::Path opts;

	template<class accel_pod_t>
	CudaRenderer(const scene_t &scene, const accel_pod_t &accel, const RenderOptions::Path &opts) : scene(scene_cuda_t(scene)), accel(accel), opts(opts) {}
	
	__device__ void render(CudaImage* image);
	__device__ Color trace(CudaRay ray, unsigned int depth = 0, curandState_t* randomState = nullptr);
	
	__device__ Vector3 sampleHemisphere(const Vector3 &X, const Vector3 &Y, const Vector3 &Z, curandState* randomState);
	
	__device__ void sampling_init(curandState_t* randomState);
	__device__ float sampling_rand(curandState_t* randomState);
};


#endif /* PRAY_CUDA_RENDERER_H */
