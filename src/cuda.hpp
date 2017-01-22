#ifndef PRAY_CUDA_H
#define PRAY_CUDA_H
#pragma once

#include <cuda_runtime_api.h>
#include <cuda.h>

namespace cuda
{
  template<class content_t>
  struct vector {
  
  // hide code from other compilers than nvcc
  #ifdef __CUDACC__
    __device__ size_t size()
    {
      return vec_size;
    }
    
    content_t& operator [](int idx)
    {
      return *(vec_pointer + sizeof(content_t) * idx);
    }
  #endif
  
  static vector<content_t> create(const std::vector<content_t> &stdvec)
  {
    vector<content_t> cudavec;
    cudavec.vec_size = stdvec.size();
    
    // alloc memory on device
    cudaMalloc((void**) &cudavec.vec_pointer, sizeof(content_t) * cudavec.vec_size);
    // copy data to device
    cudaMemcpy(cudavec.vec_pointer, stdvec.data(), sizeof(content_t) * cudavec.vec_size, cudaMemcpyHostToDevice);
      
    return cudavec;
  }
  
  void destroy()
  {
    cudaFree(vec_pointer);
		vec_pointer = 0;
		vec_size = 0;
  }

  private:
    content_t* vec_pointer;
    size_t vec_size;
  };
	
	
	template<class content_t>
	content_t* create(const content_t &obj)
	{
		content_t* ptr;
    
    // alloc memory on device
    cudaMalloc((void**) &ptr, sizeof(content_t));
    // copy data to device
    cudaMemcpy(ptr, &obj, sizeof(content_t), cudaMemcpyHostToDevice);
      
    return ptr;
	}
	
	template<class content_t>
	void destroy(content_t* ptr)
	{
		cudaFree(ptr);
	}
}

#endif // PRAY_CUDA_H
