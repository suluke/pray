#ifndef PRAY_SAMPLER_HPP
#define PRAY_SAMPLER_HPP
#pragma once

namespace sampler {
  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w == 1 && ray_t::dim::h == 1, ray_t>
      cast(const Camera &cam, const Vector3 &left, const Vector3 &top, const typename ray_t::dim_t x, const typename ray_t::dim_t y, float max_x, float max_y)
  {
    auto origin = cam.position;
    auto direction = Vector3((top * (1.f - (2 * y + 1) / max_y) + left * (1.f - (2 * x + 1) / max_x) + cam.direction).normalize());
    return {origin, direction};
  }
  
  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w != 1 || ray_t::dim::h != 1, ray_t>
      cast(const Camera &cam, const Vector3 &left, const Vector3 &top, const typename ray_t::dim_t x, const typename ray_t::dim_t y, float max_x, float max_y)
  {
    auto origin = typename ray_t::location_t(cam.position);
    auto direction = typename ray_t::vec3_t{};
    alignas(32) std::array<float, simd::REGISTER_CAPACITY_FLOAT> X;
    alignas(32) std::array<float, simd::REGISTER_CAPACITY_FLOAT> Y;
    alignas(32) std::array<float, simd::REGISTER_CAPACITY_FLOAT> Z;
    for (unsigned i = 0; i < simd::REGISTER_CAPACITY_FLOAT; ++i) {
      X[i] = cam.direction.x;
      Y[i] = cam.direction.y;
      Z[i] = cam.direction.z;
    }
    using dim = typename ray_t::dim;
    for (unsigned i_x = 0; i_x < dim::w; ++i_x) {
      float f_x = 1.f - (2 * (x + i_x) + 1) / max_x;
      auto l = left * f_x;
      for (unsigned i_y = 0; i_y < dim::h; ++i_y) {
        X[i_y * dim::w + i_x] += l.x;
        Y[i_y * dim::w + i_x] += l.y;
        Z[i_y * dim::w + i_x] += l.z;
      }
    }
    for (unsigned i_y = 0; i_y < dim::h; ++i_y) {
      float f_y = 1.f - (2 * (y + i_y) + 1) / max_y;
      auto t = top * f_y;
      for (unsigned i_x = 0; i_x < dim::w; ++i_x) {
        X[i_y * dim::w + i_x] += t.x;
        Y[i_y * dim::w + i_x] += t.y;
        Z[i_y * dim::w + i_x] += t.z;
      }
    }
    direction.x = simd::load_ps(X.data());
    direction.y = simd::load_ps(Y.data());
    direction.z = simd::load_ps(Z.data());
    direction.normalize();
    return {origin, direction};
  }
}

template<class scene_t, class tracer_t, class ray_t>
struct naive_sampler {
  static void render(const scene_t &scene, ImageView &image, tracer_t &tracer) {
    Vector3 left, right, bottom, top;
    const float aspect = (float) image.resolution.h / image.resolution.w;
    scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

    float max_x = (float) image.resolution.w;
    float max_y = (float) image.img.resolution.h;

#ifdef WITH_OMP
    #pragma omp parallel for schedule(guided, 10) collapse(2)
#endif
    for(long local_y = 0; local_y < image.resolution.h; local_y += ray_t::dim::h) {
      for(long x = 0; x < image.resolution.w; x += ray_t::dim::w) {
        auto y = image.getGlobalY(local_y);
        ray_t ray = sampler::cast<ray_t>(scene.camera, left, top, x, y, max_x, max_y);
        auto c = tracer.trace(scene, ray);
        writeColorToImage(c, image, x, local_y);
      }
    }
  }
  using dim_t = typename ray_t::dim_t;
};

template<class scene_t, class tracer_t, class ray_t>
struct interpolating_sampler {
  static void render(const scene_t &scene, ImageView &image, tracer_t &tracer) {
    Vector3 left, right, bottom, top;
    const float aspect = (float) image.resolution.h / image.resolution.w;
    scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

    float max_x = (float) image.resolution.w;
    float max_y = (float) image.img.resolution.h;

#ifdef WITH_OMP
    #pragma omp parallel for schedule(guided, 10) collapse(2)
#endif
    for (long local_y = 0; local_y < image.resolution.h; local_y += ray_t::dim::h) {
      for (long x = 0; x < image.resolution.w; x += ray_t::dim::w) {
        // FIXME the current modulo checks will break the image for SSERays
        // because the loop stride is not 1 any more but 2 or 4
        if (x == 0 || x == image.resolution.w-1 || local_y == 0 || image.resolution.h - 1 == local_y ||
           (x%2 == 0 && local_y%2 == 1) || (x%2 == 1 && local_y%2 == 0))
        {
          auto y = image.getGlobalY(local_y);
          ray_t ray = sampler::cast<ray_t>(scene.camera, left, top, x, y, max_x, max_y);
          auto c = tracer.trace(scene, ray);
          writeColorToImage(c, image, x, local_y);
        }
      }
    }
#ifdef WITH_OMP
    #pragma omp parallel for collapse(2)
#endif
    for(long y = 1; y < image.resolution.h-1; ++y) {
      for(long x = 1; x < image.resolution.w-1; ++x) {
        if (x == 0 || x == image.resolution.w-1 || y == 0 || image.resolution.h -1 == y || (x%2 == 0 && y%2 == 1) || (x%2 == 1 && y%2 == 0)) {
          continue;
        } else {
          Color c = image.getPixel(x,y);
          c += image.getPixel(x-1,y) * 0.25f;
          c += image.getPixel(x,y-1) * 0.25f;
          c += image.getPixel(x+1,y) * 0.25f;
          c += image.getPixel(x,y+1) * 0.25f;
          image.setPixel(x, y, c);
        }
      }
    }
  }
};

#endif // PRAY_SAMPLER_HPP
