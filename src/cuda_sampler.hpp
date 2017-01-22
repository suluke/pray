#ifndef PRAY_CUDA_SAMPLER_HPP
#define PRAY_CUDA_SAMPLER_HPP
#pragma once

template<class scene_t, class tracer_t>
struct naive_sampler {
  using ray_t = Ray<scene_t>;
  
  static void render(const scene_t &scene, ImageView &image, tracer_t &tracer) {
    Vector3 left, right, bottom, top;
    const float aspect = (float) image.resolution.h / image.resolution.w;
    scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

    float max_x = (float) image.resolution.w;
    float max_y = (float) image.img.resolution.h;

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

#endif // PRAY_CUDA_SAMPLER_HPP
