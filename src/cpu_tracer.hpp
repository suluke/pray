#ifndef PRAY_NAIVE_TRACER_H
#define PRAY_NAIVE_TRACER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"

template<class AccelerationStructure>
class CpuTracer {
  const Scene &scene;
  AccelerationStructure acceleration_structure;
public:
  CpuTracer(const Scene &scene) : scene(scene) {}
  void preprocess();
  void render(ImageView &image) const;
};

#include "cpu_tracer.inl"

#endif
