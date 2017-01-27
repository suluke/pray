#ifndef PRAY_CUDA_H
#define PRAY_CUDA_H
#pragma once

#include <cuda_runtime_api.h>
#include <cuda.h>

#include <exception>
#include <string>
#include <sstream>

#ifdef __CUDACC__
	#define __cuda__ __device__ __host__
#else
	#define __cuda__
#endif


namespace cuda
{
  inline void checkForError(const char* file, const char* func, int line)
  {
    // error handling
    cudaError_t error = cudaGetLastError();
    if (error)
    {
      std::stringstream msg; 
      msg << "CUDA error occured: " << cudaGetErrorString(error) << " (" << file << ":" << line << ":" << func << ")\n";
      throw std::runtime_error(msg.str().c_str());
    }
    
    
  }
  
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
		
		std::cout << "size of array: " << cudavec.vec_size << "\n";
		
    // alloc memory on device
    cudaMalloc((void**) &cudavec.vec_pointer, sizeof(content_t) * cudavec.vec_size);
    cuda::checkForError(__FILE__, __func__, __LINE__);
    // copy data to device
    cudaMemcpy(cudavec.vec_pointer, stdvec.data(), sizeof(content_t) * cudavec.vec_size, cudaMemcpyHostToDevice);
    cuda::checkForError(__FILE__, __func__, __LINE__);
      
    return cudavec;
  }
  
  void destroy()
  {
    cudaFree(vec_pointer);
    cuda::checkForError(__FILE__, __func__, __LINE__);
		vec_pointer = 0;
		vec_size = 0;
  }

  private:
    content_t* vec_pointer;
    typename std::vector<content_t>::size_type vec_size;
  };
	
	
	template<class content_t>
	content_t* create(const content_t &obj)
	{
		content_t* ptr;
    
    // alloc memory on device
    cudaMalloc((void**) &ptr, sizeof(content_t));
    cuda::checkForError(__FILE__, __func__, __LINE__);
    // copy data to device
    cudaMemcpy(ptr, &obj, sizeof(content_t), cudaMemcpyHostToDevice);
    cuda::checkForError(__FILE__, __func__, __LINE__);
		
    return ptr;
	}
	
	template<class content_t>
	void destroy(content_t* ptr)
	{
		cudaFree(ptr);
    cuda::checkForError(__FILE__, __func__, __LINE__);
	}
}

#endif // PRAY_CUDA_H
