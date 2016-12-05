#ifndef PRAY_NAIVE_TRACER_H
#define PRAY_NAIVE_TRACER_H
#pragma once

#include "scene.hpp"

class NaiveTracer {
  const Scene &scene;
public:
  NaiveTracer(const Scene &scene) : scene(scene) {}
  Color trace(const Ray &ray) const;
};

#endif
