#ifndef PRAY_CUDA_IMAGE_H
#define PRAY_CUDA_IMAGE_H
#pragma once

#include "cuda_lib.hpp"
#include "image.hpp"
#include "math.hpp"

#include <exception>
#include <cuda_runtime_api.h>



struct CudaImage
{
	using dim_t = Image::dim_t;
	
  // reference for copying back the resulting image afterwards
  ImageView &image;
  
	uint8_t* pixels = nullptr; // device pointer
	dim_t min_y;
	dim_t max_y;
	const IntDimension2 viewResolution; // of view
	const IntDimension2 resolution;     // of complete image
  
	CudaImage();
	CudaImage(const CudaImage&);
	CudaImage(const CudaImage&&);
	
  CudaImage(ImageView &image) : image(image), min_y(image.min_y), max_y(image.max_y), viewResolution(image.resolution), resolution(image.img.resolution)
	{
    // allocate memory for image
		unsigned int pixels_size = sizeof(uint8_t) * 3 * viewResolution.w * viewResolution.h;
		cudaMalloc((void**) &pixels, pixels_size);
    cuda::checkForError(__FILE__, __func__, __LINE__);
	}
	
	~CudaImage()
	{
		// free memory
		cudaFree(pixels);
    cuda::checkForError(__FILE__, __func__, __LINE__);
		pixels = nullptr;
	}	
	
	void copyBack()
	{
		if (pixels == nullptr)
			throw std::runtime_error("CudaImage object was already destroyed");
		
		// copy resulting image back
		uint8_t* pixels_start = image.img.pixels.data() + sizeof(uint8_t) * 3 * resolution.w * min_y;
		unsigned int pixels_size = sizeof(uint8_t) * 3 * viewResolution.w * viewResolution.h;
		
		cudaMemcpy(pixels_start, pixels, pixels_size, cudaMemcpyDeviceToHost);
    cuda::checkForError(__FILE__, __func__, __LINE__);
		
		#ifdef WITH_PROGRESS
			image.img.writtenPixels += viewResolution.w * viewResolution.h;
		#endif
	}
	
	__device__ void setPixel(dim_t x, dim_t y, const Color &c)
	{
		pixels[3 * (y * viewResolution.w + x) + 0] = lroundf(min(1.f, c.r) * 255.f);
		pixels[3 * (y * viewResolution.w + x) + 1] = lroundf(min(1.f, c.g) * 255.f);
		pixels[3 * (y * viewResolution.w + x) + 2] = lroundf(min(1.f, c.b) * 255.f);		
	}

	__device__ dim_t getGlobalY(dim_t y) {
		return y + min_y;
	}
};

#endif
