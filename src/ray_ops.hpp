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
  static inline bool getEmission(ray_intersect intersect, const scene_t &scene, ray_color *out_color) {
    const auto &triangle = scene.triangles[intersect];
    const auto &material = scene.materials[triangle.material_index];
    if (material.isEmission) {
      *out_color = material.color;
      return true;
    }
    *out_color = {0, 0, 0};
    return false; 
  }


  // =============== SSERay ===============
  using sse_ray_loc = simd::Vec3Pack;
  using sse_ray_vec3 = simd::Vec3Pack;
  using sse_ray_intersect = simd::intty;
  using sse_ray_color = SSEColor;
  using sse_ray_bool = simd::intty;
  
  template <class scene_t, class rand_t>
  static inline SSERay<scene_t> sampleHemisphere(const sse_ray_loc &origin, const sse_ray_vec3 &X, const sse_ray_vec3 &Y, const sse_ray_vec3 &Z, rand_t &sampling_rand) {
    float u1 = sampling_rand();
    float u2 = sampling_rand();
    float r = std::sqrt(1.f - u1);
    float phi = 2 * std::acos(-1.f) * u2;
    float x = std::cos(phi) * r;
    float y = std::sin(phi) * r;
    float z = std::sqrt(u1);
    return SSERay<scene_t>(origin, X * x + Y * y + Z * z);
  }

  template <class scene_t>
  static inline void getIntersectionBase(sse_ray_vec3 &N, sse_ray_intersect intersect, const scene_t &scene, sse_ray_vec3 *out_X, sse_ray_vec3 *out_Y, sse_ray_vec3 *out_Z) {
    alignas(32) std::array<TriangleIndex, simd::REGISTER_CAPACITY_I32> triangles;
    simd::store_si((simd::intty *) triangles.data(), intersect);
    alignas(32) std::array<float, simd::REGISTER_CAPACITY_FLOAT> X_x, X_y, X_z;
    for (unsigned i = 0; i < triangles.size(); ++i) {
      auto T_idx = triangles[i];
      if (T_idx != TriangleIndex_Invalid) {
        auto &triangle = scene.triangles[T_idx];
        auto X = (triangle.vertices[1] - triangle.vertices[0]).normalize();
        X_x[i] = X.x;
        X_y[i] = X.y;
        X_z[i] = X.z;
      } else {
        X_x[i] = 0;
        X_y[i] = 0;
        X_z[i] = 0;
      }
    }
    *out_X = {simd::load_ps(X_x.data()), simd::load_ps(X_y.data()), simd::load_ps(X_z.data())};
    *out_Y = N.cross(*out_X).normalize(); // TODO this will be INFINITY (division by zero) for invalid triangles
    *out_Z = N;
  }

  template <class scene_t>
  static inline sse_ray_bool getEmission(sse_ray_intersect intersect, const scene_t &scene, sse_ray_color *out_color) {
    alignas(32) std::array<TriangleIndex, simd::REGISTER_CAPACITY_I32> triangles;
    simd::store_si((simd::intty *) triangles.data(), intersect);
    alignas(32) std::array<float, simd::REGISTER_CAPACITY_FLOAT> R, G, B;
    alignas(32) std::array<int32_t, simd::REGISTER_CAPACITY_I32> IsEmission;
    for (unsigned i = 0; i < triangles.size(); ++i) {
      auto T_idx = triangles[i];
      if (T_idx != TriangleIndex_Invalid) {
        const auto &triangle = scene.triangles[T_idx];
        const auto &material = scene.materials[triangle.material_index];
        if (material.isEmission) {
          auto &c = material.color;
          R[i] = c.r;
          G[i] = c.g;
          B[i] = c.b;
          IsEmission[i] = true;
        } else {
          R[i] = 0;
          G[i] = 0;
          B[i] = 0;
          IsEmission[i] = false;
        }
      } else {
        R[i] = scene.background_color.r;
        G[i] = scene.background_color.g;
        B[i] = scene.background_color.b;
        IsEmission[i] = true;
      }
    }
    *out_color = {simd::load_ps(R.data()), simd::load_ps(G.data()), simd::load_ps(B.data())};
    return simd::load_si((sse_ray_bool *) IsEmission.data());
  }
}

#endif // PRAY_RAY_OPS_HPP
