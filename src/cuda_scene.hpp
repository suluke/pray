#ifndef PRAY_CUDA_SCENE_H
#define PRAY_CUDA_SCENE_H
#pragma once

#include "math.hpp"
#include "scene.hpp"
#include "cuda.h"

#include <cstdint>
#include <limits>
#include <array>
#include <vector>
#include <memory>

using TriangleIndex = uint32_t;
using MaterialIndex = uint32_t;

template<class material_t>
struct CudaScene
{
	using scene_t = Scene<material_t>;
	
  scene_t &scene;
  
	cuda::vector<Triangle> triangles;
	cuda::vector<material_t> materials;

	Camera* camera;
	Color* background_color;
  
  CudaScene(scene_t) : scene(scene) {};
  
  void initialize()
  {
    // allocate memory and copy data
    triangles = cuda::vector<Triangle>::create(scene.triangles);
    materials = cuda::vector<material_t>::create(scene.materials);
    camera = cuda::create<Camera>(scene.camera);
		background_color = cuda::create<Color>(scene.background_color);
  }

  void finalize()
  {
    // free memory
    triangles.destroy();
    materials.destroy();
    cuda::destroy<Camera>(camera);
		cuda::destroy<Color>(background_color);
  }
};

#endif
