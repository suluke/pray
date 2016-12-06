#ifndef PRAY_NAIVE_TRACER_H
#define PRAY_NAIVE_TRACER_H
#pragma once

#include "scene.hpp"
#include "image.hpp"

class CpuTracer {
  const Scene &scene;
public:
  CpuTracer(const Scene &scene) : scene(scene) {}
  void render(ImageView &image) const;
};

#endif
