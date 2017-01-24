#ifndef PRAY_DEBUG_H
#define PRAY_DEBUG_H
#pragma once

#ifndef NDEBUG
#define DEBUG
#endif

#include <iostream>

#ifdef DEBUG
	#ifdef __CUDACC__
		#warning "Asserts disabled because of CUDA"
		#define ASSERT(exp) do {} while(false)
	#else
		#define ASSERT(exp) do { if(!(exp)) std::cerr << "Assertion failed: " << #exp " at(" __FILE__ ":" << __LINE__ << ")\n"; } while(false)
	#endif
#else
	#define ASSERT(exp) do {} while(false)
#endif

#endif
