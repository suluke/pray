#ifndef PRAY_CUDA_PATHTRACER_TRACER_H
#define PRAY_CUDA_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "ray.hpp"
#include "bih.hpp"
#include "cuda.hpp"
#include "cuda_ray.hpp"


struct CudaImageView
{
	using dim_t = Image::dim_t;
	
	// this reference will be useless after copied to the device
	ImageView image;
	
	uint8_t* pixels;
	dim_t min_y;
	dim_t max_y;
	const IntDimension2 resolution;     // of view
	const IntDimension2 resolution_org; // of orginal image
	
	CudaImageView(ImageView &image) : image(image), min_y(image.min_y), max_y(image.max_y), resolution(image.resolution), resolution_org(image.img.resolution) {}
	
	void initialize()
	{
		unsigned int pixels_size = sizeof(uint8_t) * 3 * image.resolution.w * image.resolution.h;
		cudaMalloc((void**) &pixels, pixels_size);
	}
	
	void finalize()
	{
		uint8_t* pixels_start = image.img.pixels.data() + sizeof(uint8_t) * 3 * image.resolution.w * image.min_y;
		unsigned int pixels_size = sizeof(uint8_t) * 3 * image.resolution.w * image.resolution.h;
		cudaMemcpy(pixels_start, pixels, pixels_size, cudaMemcpyDeviceToHost);
		cudaFree(pixels);
	}

	#ifdef __CUDACC__
		__device__ void setPixel(dim_t x, dim_t y, const Color &c)
		{
			pixels[3 * (y * resolution.w + x) + 0] = c.r;
			pixels[3 * (y * resolution.w + x) + 1] = c.g;
			pixels[3 * (y * resolution.w + x) + 2] = c.b;
		}

		__device__ Color getPixel(dim_t x, dim_t y) {
			return Color{((float)pixels[3 * (y * resolution.w + x) + 0]) / 255.f,
									((float)pixels[3 * (y * resolution.w + x) + 1]) / 255.f,
									((float)pixels[3 * (y * resolution.w + x) + 2]) / 255.f};
		}

		__device__ dim_t getGlobalY(dim_t y) {
			return y + min_y;
		}
	#endif
};


struct CudaPathTracer {
	using scene_t = PathScene;
  using accel_t = BihPOD<scene_t>;
  using ray_t = CudaRay<scene_t>;
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
	Camera* d_camera;

  CudaPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &bih) : scene(scene), bih(bih), opts(opts) {}
	void render(ImageView &image);
	
	void initialize();
	void finalize();
	
	#ifdef __CUDACC__
    __device__ void d_render(CudaImageView image);
	#endif

private:
	#ifdef __CUDACC__
		__device__ typename ray_t::color_t d_trace(ray_t ray, unsigned depth = 0);
	#endif
};

#endif /* PRAY_CUDA_PATHTRACER_TRACER_H */
