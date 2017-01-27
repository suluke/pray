#ifndef PRAY_CUDA_PATHTRACER_TRACER_H
#define PRAY_CUDA_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "ray.hpp"
#include "bih.hpp"

#include "cuda_bih.hpp"
#include "cuda_scene.hpp"

template<class ray_t, class accel_t>
struct CudaPathTracer {
	using scene_t = PathScene;
	using scene_cuda_t = CudaScene;
	using accel_cuda_t = CudaBih<scene_t>;
  using accel_pod_t = BihPOD<scene_t>;
	using renderopts_t = RenderOptions::Path;
  //using ray_t = CudaRay<CudaScene>;
	using material_t = EmissionMaterial;
	
	scene_cuda_t scene;
	accel_cuda_t accel;
	
	const renderopts_t &opts;

  CudaPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &accel) : scene(scene_cuda_t(scene)), accel(accel_cuda_t(accel.pod)), opts(opts) {}
	void render(ImageView &image);
	
	void initialize();
	void finalize();
};


#endif /* PRAY_CUDA_PATHTRACER_TRACER_H */
