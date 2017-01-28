#ifndef PRAY_CUDA_PATHTRACER_TRACER_H
#define PRAY_CUDA_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "ray.hpp"
#include "bih.hpp"

struct CudaPathTracer {
  using accel_t = BihPOD<PathScene>;
  using ray_t = Ray<PathScene>;
  
  const PathScene &scene;
  const RenderOptions::Path &opts;
  const accel_t &acceleration_structure;

  CudaPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &acceleration_structure) : scene(scene), opts(opts), acceleration_structure(acceleration_structure) {}
  typename Ray<PathScene>::color_t trace(const PathScene &scene, const Ray<PathScene> &ray, unsigned depth = 0) const;

private:
  // Relevant: https://github.com/s9w/articles/blob/master/perf%20cpp%20random.md
  //std::function<float()> sampling_rand = std::bind(std::uniform_real_distribution<float>(0, 1), std::default_random_engine());
  typename ray_t::vec3_t sampleHemisphere(const typename ray_t::vec3_t &X, const typename ray_t::vec3_t &Y, const typename ray_t::vec3_t &Z) const;
};

//#include "cuda_pathtracer.impl.hpp"

#endif /* PRAY_CUDA_PATHTRACER_TRACER_H */
