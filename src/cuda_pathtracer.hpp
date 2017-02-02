#ifndef PRAY_CUDA_PATHTRACER_TRACER_H
#define PRAY_CUDA_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"

template<class scene_t>
class DummyAccelerationPOD;
template<class scene_t>
class BihPOD;
template<class scene_t>
class KdTreePOD;

struct CudaPathTracer {
	virtual ~CudaPathTracer() = default;
	
	virtual void render(ImageView &image) = 0;
	static std::unique_ptr<CudaPathTracer> Create(const PathScene &, const RenderOptions::Path &, const DummyAccelerationPOD<PathScene> &);
	static std::unique_ptr<CudaPathTracer> Create(const PathScene &, const RenderOptions::Path &, const BihPOD<PathScene> &);
	static std::unique_ptr<CudaPathTracer> Create(const PathScene &, const RenderOptions::Path &, const KdTreePOD<PathScene> &);
};


#endif /* PRAY_CUDA_PATHTRACER_TRACER_H */
