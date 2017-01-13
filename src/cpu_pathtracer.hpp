#ifndef PRAY_CPU_PATHTRACER_TRACER_H
#define PRAY_CPU_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"
#include <functional>
#include <random>

template<class ray_t, class accel_t>
struct CpuPathTracer {
  const PathScene &scene;
  const RenderOptions::Path &opts;
  const accel_t &acceleration_structure;

  CpuPathTracer(const PathScene &scene, const RenderOptions::Path &opts, const accel_t &acceleration_structure) : scene(scene), opts(opts), acceleration_structure(acceleration_structure) {}
  void render(ImageView &image) const;

private:
  // Relevant: https://github.com/s9w/articles/blob/master/perf%20cpp%20random.md
  std::function<float()> sampling_rand = std::bind(std::uniform_real_distribution<float>(0, 1), std::default_random_engine());

  typename ray_t::color_t trace(const PathScene &scene, const ray_t &ray, unsigned depth = 0) const;
};

#include "cpu_pathtracer.impl.hpp"

#endif /* PRAY_CPU_PATHTRACER_TRACER_H */
