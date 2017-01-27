#ifndef PRAY_CUDA_IMAGE_H
#define PRAY_CUDA_IMAGE_H
#pragma once

#include "image.hpp"


struct CudaImage
{
	using dim_t = Image::dim_t;
	
  // reference for copying back the resulting image afterwards
  ImageView &image;
  
	uint8_t* pixels;
	dim_t min_y;
	dim_t max_y;
	const IntDimension2 viewResolution; // of view
	const IntDimension2 resolution;     // of complete image
  
  CudaImage(ImageView &image) : image(image), min_y(image.min_y), max_y(image.max_y), viewResolution(image.resolution), resolution(image.img.resolution) {}
	
	void initialize()
	{
    // allocate memory for image
		unsigned int pixels_size = sizeof(uint8_t) * 3 * resolution.w * resolution.h;
		cudaMalloc((void**) &pixels, pixels_size);
    cuda::checkForError(__FILE__, __func__, __LINE__);
	}
	
	void finalize()
	{
    // copy resulting image back
		uint8_t* pixels_start = image.img.pixels.data() + sizeof(uint8_t) * 3 * image.resolution.w * image.min_y;
		unsigned int pixels_size = sizeof(uint8_t) * 3 * resolution.w * resolution.h;
		cudaMemcpy(pixels_start, pixels, pixels_size, cudaMemcpyDeviceToHost);
    cuda::checkForError(__FILE__, __func__, __LINE__);
		cudaFree(pixels);
    cuda::checkForError(__FILE__, __func__, __LINE__);
	}
};

/*

#include "cuda_math.hpp"
#include "cuda_renderdata.hpp"

using dim_t = CudaImage::dim_t;



__device__ void imageSetPixel(CudaRenderData &data, dim_t x, dim_t y, const CudaColor &c)
{
  data.d_pixels[3 * (y * data.viewResolution.w + x) + 0] = lroundf(min(1.f, c.r) * 255.f);
  data.d_pixels[3 * (y * data.viewResolution.w + x) + 1] = lroundf(min(1.f, c.g) * 255.f);
  data.d_pixels[3 * (y * data.viewResolution.w + x) + 2] = lroundf(min(1.f, c.b) * 255.f);
}

__device__ dim_t imageGetGlobalY(CudaRenderData &data, dim_t y) {
  return y + data.min_y;
}


void initialize()
{
  unsigned int pixels_size = sizeof(uint8_t) * 3 * resolution.w * resolution.h;
  cudaMalloc((void**) &d_pixels, pixels_size);
  cuda::checkForError(__FILE__, __func__, __LINE__);
}

void finalize()
{
  uint8_t* pixels_start = image.img.pixels.data() + sizeof(uint8_t) * 3 * image.resolution.w * image.min_y;
  unsigned int pixels_size = sizeof(uint8_t) * 3 * resolution.w * resolution.h;
  cudaMemcpy(pixels_start, d_pixels, pixels_size, cudaMemcpyDeviceToHost);
  cuda::checkForError(__FILE__, __func__, __LINE__);
  cudaFree(d_pixels);
  cuda::checkForError(__FILE__, __func__, __LINE__);
}
*/

#endif
