#ifndef PRAY_CPU_PATHTRACER_TRACER_H
#define PRAY_CPU_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"
#include "ray.hpp"
#include "sse_ray.hpp"
#include <functional>
#include <random>

template<class ray_t, class accel_t>
struct CpuPathTracer {
  const PathScene &scene;
  const RenderOptions::Path &opts;
  const accel_t &acceleration_structure;

  CpuPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &acceleration_structure) : scene(scene), opts(opts), acceleration_structure(acceleration_structure) {}
  typename Ray<PathScene>::color_t trace(const PathScene &scene, const Ray<PathScene> &ray, unsigned depth = 0) const;
  typename SSERay<PathScene>::color_t trace(const PathScene &scene, const SSERay<PathScene> &ray, unsigned depth = 0, typename SSERay<PathScene>::mask_t mask = simd::set1_epi32(-1)) const;

private:
  // Relevant: https://github.com/s9w/articles/blob/master/perf%20cpp%20random.md
  std::function<float()> sampling_rand = std::bind(std::uniform_real_distribution<float>(0, 1), std::default_random_engine());
  typename ray_t::vec3_t sampleHemisphere(const typename ray_t::vec3_t &X, const typename ray_t::vec3_t &Y, const typename ray_t::vec3_t &Z) const;
#ifndef WITH_CHEATS
  typename Ray<PathScene>::vec3_t    sampleHemisphereImpl(const typename Ray<PathScene>::vec3_t &X, const typename Ray<PathScene>::vec3_t &Y, const typename Ray<PathScene>::vec3_t &Z) const;
  typename SSERay<PathScene>::vec3_t sampleHemisphereImpl(const typename SSERay<PathScene>::vec3_t &X, const typename SSERay<PathScene>::vec3_t &Y, const typename SSERay<PathScene>::vec3_t &Z) const;
#endif
};

#include "cpu_pathtracer.impl.hpp"

#endif /* PRAY_CPU_PATHTRACER_TRACER_H */
