#ifndef PRAY_CPU_PATHTRACER_TRACER_H
#define PRAY_CPU_PATHTRACER_TRACER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"

template<class ray_t, class accel_t>
struct CpuPathTracer {
  const Scene &scene;
  const RenderOptions::Path &opts;
  const accel_t &acceleration_structure;

  CpuPathTracer(const Scene &scene, const RenderOptions::Path &opts, const accel_t &acceleration_structure) : scene(scene), opts(opts), acceleration_structure(acceleration_structure) {}
  void render(ImageView &image) const;

private:
  typename ray_t::color_t trace(const Scene &scene, const ray_t &ray) const;
};

#include "cpu_pathtracer.impl.hpp"

#endif /* PRAY_CPU_PATHTRACER_TRACER_H */
