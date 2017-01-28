#ifndef PRAY_CUDA_H
#define PRAY_CUDA_H
#pragma once

#ifdef __CUDACC__
	#define __cuda__ __device__ __host__
#else
	#define __cuda__
#endif

#endif // PRAY_CUDA_H
