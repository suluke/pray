#ifndef PRAY_CUDA_LIB_H
#define PRAY_CUDA_LIB_H
#pragma once

#include <cuda_runtime_api.h>
#include <cuda.h>

#include <exception>
#include <string>
#include <sstream>

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
	public:
		// hide code from other compilers than nvcc
		__device__ size_t size() const
		{
			return vec_size;
		}
		
		__device__ content_t* data() const
		{
			return vec_pointer;
		}
		
		__device__ const content_t& operator [](int idx) const
		{
			ASSERT(vec_pointer != nullptr);
			return *(vec_pointer + idx);
		}
		
		static vector<content_t> create(const std::vector<content_t> &stdvec)
		{
			vector<content_t> cudavec;
			cudavec.vec_size = stdvec.size();
			
			// alloc memory on device
			cudaMalloc((void**) &cudavec.vec_pointer, sizeof(content_t) * stdvec.size());
			cuda::checkForError(__FILE__, __func__, __LINE__);
			// copy data to device
			cudaMemcpy(cudavec.vec_pointer, stdvec.data(), sizeof(content_t) * stdvec.size(), cudaMemcpyHostToDevice);
			cuda::checkForError(__FILE__, __func__, __LINE__);
				
			return std::move(cudavec);
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
		ASSERT(ptr != nullptr);
		cudaFree(ptr);
    cuda::checkForError(__FILE__, __func__, __LINE__);
	}
	
	template<class T> 
  __device__ inline void swap(T &a, T &b)
	{
		T c(a);
		a=b;
		b=c;
	}
	
	template<class T>
	__device__ inline const T& min(const T &a, const T &b) {
		return (a < b) ? a : b;
	}
	
	template<class T>
	__device__ inline const T& max(const T &a, const T &b) {
		return (a < b) ? b : a;
	}
}

#endif // PRAY_CUDA_LIB_H
