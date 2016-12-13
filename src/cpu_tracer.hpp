#ifndef PRAY_NAIVE_TRACER_H
#define PRAY_NAIVE_TRACER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"

template<class ray_t>
class CpuTracer {
  const Scene &scene;
public:
  CpuTracer(const Scene &scene) : scene(scene) {}
  void render(ImageView &image) const;

private:
  typename ray_t::intersect_t intersect(const Scene &scene, const ray_t &ray, typename ray_t::distance_t *out_distance) const;
  typename ray_t::color_t trace(const Scene &scene, const ray_t &ray) const;
};

#include "cpu_tracer.impl.hpp"

#endif
