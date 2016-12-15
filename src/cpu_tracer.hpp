#ifndef PRAY_NAIVE_TRACER_H
#define PRAY_NAIVE_TRACER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"

template<class ray_t, class accel_t>
class CpuTracer {
  const Scene &scene;
  accel_t acceleration_structure;
public:
  CpuTracer(const Scene &scene) : scene(scene) {}
  void preprocess();
  void render(ImageView &image) const;

private:
  typename ray_t::color_t trace(const Scene &scene, const ray_t &ray) const;
};

#include "cpu_tracer.impl.hpp"

#endif
