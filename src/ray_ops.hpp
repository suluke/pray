#ifndef PRAY_RAY_OPS_HPP
#define PRAY_RAY_OPS_HPP
#pragma once

#include "ray.hpp"
#include "sse_ray.hpp"

namespace ray_ops {
  using ray_loc = Vector3;
  using ray_vec3 = Vector3;
  using ray_intersect = TriangleIndex;
  using ray_color = Color;

  template <class scene_t, class rand_t>
  static inline Ray<scene_t> sampleHemisphere(const ray_loc &origin, const ray_vec3 &X, const ray_vec3 &Y, const ray_vec3 &Z, rand_t &sampling_rand) {
    float u1 = sampling_rand();
    float u2 = sampling_rand();
    float r = std::sqrt(1.f - u1);
    float phi = 2 * std::acos(-1.f) * u2;
    float x = std::cos(phi) * r;
    float y = std::sin(phi) * r;
    float z = std::sqrt(u1);
    return Ray<scene_t>(origin, X * x + Y * y + Z * z);
  }

  template <class scene_t>
  static inline void getIntersectionBase(ray_vec3 &N, ray_intersect intersect, const scene_t &scene, ray_vec3 *out_X, ray_vec3 *out_Y, ray_vec3 *out_Z) {
    const auto &triangle = scene.triangles[intersect];
    *out_X = (triangle.vertices[1] - triangle.vertices[0]).normalize();
    *out_Y = N.cross(*out_X).normalize();
    *out_Z = N;
  }

  template <class scene_t>
  static inline bool getEmission(ray_intersect intersected_triangle, const scene_t &scene, ray_color *out_color) {
    const auto &triangle = scene.triangles[intersected_triangle];
    const auto &material = scene.materials[triangle.material_index];
    if (material.isEmission) {
      *out_color = material.color;
      return true;
    }
    *out_color = {0, 0, 0};
    return false; 
  }
}

#endif // PRAY_RAY_OPS_HPP
