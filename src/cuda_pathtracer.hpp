#ifndef PRAY_CUDA_PATHTRACER_TRACER_H
#define PRAY_CUDA_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "ray.hpp"
#include "bih.hpp"
#include "cuda.hpp"


struct CudaPathTracer {
	using scene_t = PathScene;
  using accel_t = BihPOD<scene_t>;
  using ray_t = Ray<scene_t>;
	using material_t = EmissionMaterial;
	
	using Node = accel_t::Node;
  
	// references (cannot be used in __device__ functions)
  const PathScene &scene;
  const accel_t &bih;
	
	// RenderOptions will be copied to device!
	const RenderOptions::Path opts;
	
	// device data (not valid on cpu!)
	AABox3* d_scene_aabb;
	cuda::vector<Node> d_nodes;
	cuda::vector<Triangle> d_triangles;
	cuda::vector<material_t> d_materials;

  CudaPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &bih) : scene(scene), bih(bih), opts(opts) {}
	void render(ImageView &image);
	
	void initialize();
	void finalize();
	
	#ifdef __CUDACC__
    __device__ void d_render();
	#endif

private:
	#ifdef __CUDACC__
		// more __device__ functions
	#endif
};

#endif /* PRAY_CUDA_PATHTRACER_TRACER_H */
