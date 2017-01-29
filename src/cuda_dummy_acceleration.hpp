#ifndef PRAY_CUDA_DUMMY_ACCELERATION_H
#define PRAY_CUDA_DUMMY_ACCELERATION_H
#pragma once

#include "cuda_lib.hpp"
#include "cuda_scene.hpp"
#include "cuda_ray.hpp"
#include "scene.hpp"

template<class accel_t>
struct CudaDummyAcceleration
{
	using scene_t = CudaScene;
	using ray_t = CudaRay;
	
	CudaDummyAcceleration(const accel_t &accel) {}
	
	// for calculation of stack size need
	static const size_t stack_size = 0;
	
	__device__ ray_t::intersect_t intersect(const scene_t &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const;
};

#endif
