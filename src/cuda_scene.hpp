#ifndef PRAY_CUDA_SCENE_H
#define PRAY_CUDA_SCENE_H
#pragma once

#include "math.hpp"
#include "scene.hpp"
#include "cuda_lib.hpp"

#include <cstdint>
#include <limits>
#include <array>
#include <vector>
#include <memory>

#include <cuda_runtime_api.h>

using TriangleIndex = uint32_t;
using MaterialIndex = uint32_t;

struct CudaScene
{
	using scene_t = PathScene;
	using material_t = EmissionMaterial;
  
	cuda::vector<Triangle> triangles;
	cuda::vector<material_t> materials;

	Camera* camera;
	Color* background_color;
	
	CudaScene(const CudaScene&);
	CudaScene(const CudaScene&&);
  
  CudaScene(const scene_t &scene)
	{
		// allocate memory and copy data
    triangles = cuda::vector<Triangle>::create(scene.triangles);
    materials = cuda::vector<material_t>::create(scene.materials);
    camera = cuda::create<Camera>(scene.camera);
		background_color = cuda::create<Color>(scene.background_color);
	};

  ~CudaScene()
  {
    // free memory
    triangles.destroy();
    materials.destroy();
    cuda::destroy<Camera>(camera);
		cuda::destroy<Color>(background_color);
  }
};

#endif
