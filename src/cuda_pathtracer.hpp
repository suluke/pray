#ifndef PRAY_CUDA_PATHTRACER_TRACER_H
#define PRAY_CUDA_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "ray.hpp"
#include "bih.hpp"

#include "cuda_bih.hpp"

template<class ray_t, class accel_t>
struct CudaPathTracer {
	using scene_t = PathScene;
	using accel_cuda_t = CudaBih<scene_t>;
  using accel_pod_t = BihPOD<scene_t>;
  //using ray_t = CudaRay<CudaScene>;
	using material_t = EmissionMaterial;
	
	scene_t scene;
	accel_pod_t accel;
	
	const RenderOptions::Path &opts;

  CudaPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &accel) : scene(scene), accel(accel.pod), opts(opts) {}
	void render(ImageView &image);
	
	void initialize();
	void finalize();
};

#endif /* PRAY_CUDA_PATHTRACER_TRACER_H */
