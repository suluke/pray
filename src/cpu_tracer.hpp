#ifndef PRAY_NAIVE_TRACER_H
#define PRAY_NAIVE_TRACER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"

template<class ray_t, class accel_t>
struct CpuTracer {
  const WhittedScene &scene;
  const accel_t &acceleration_structure;

  CpuTracer(const WhittedScene &scene, const accel_t &acceleration_structure) : scene(scene), acceleration_structure(acceleration_structure) {}
  typename ray_t::color_t trace(const WhittedScene &scene, const ray_t &ray) const;
};

#include "cpu_tracer.impl.hpp"

#endif
