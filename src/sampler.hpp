#ifndef PRAY_SAMPLER_HPP
#define PRAY_SAMPLER_HPP
#pragma once

namespace sampler {
template <class ray_t>
static inline std::enable_if_t<ray_t::dim::w == 1 && ray_t::dim::h == 1, ray_t>
cast(const Camera &cam, const Vector3 &left, const Vector3 &top,
     const typename ray_t::dim_t x, const typename ray_t::dim_t y, float max_x,
     float max_y) {
  auto origin = cam.position;
  auto direction = Vector3((top * (1.f - (2 * y + 1) / max_y) +
                            left * (1.f - (2 * x + 1) / max_x) + cam.direction)
                               .normalize());
  return {origin, direction};
}

template <class ray_t>
static inline std::enable_if_t<ray_t::dim::w != 1 || ray_t::dim::h != 1, ray_t>
cast(const Camera &cam, const Vector3 &left, const Vector3 &top,
     const typename ray_t::dim_t x, const typename ray_t::dim_t y, float max_x,
     float max_y) {
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

/// If cast in sparse mode, ray dimensions change
template <typename dim> struct sparse_dim {
  static const unsigned w = (dim::w <= dim::h ? dim::w * 2 : dim::w);
  static const unsigned h = (dim::h < dim::w ? dim::h * 2 : dim::h);
};

template <class ray_t>
static inline std::enable_if_t<ray_t::dim::w == 1 && ray_t::dim::h == 1, ray_t>
sparse_cast(const Camera &cam, const Vector3 &left, const Vector3 &top,
            typename ray_t::dim_t x, const typename ray_t::dim_t y, float max_x,
            float max_y, bool inverse = false) {
  // For odd rows, we cast the ray for the right of two neighboring pixels
  if ((y % 2 == 1 && x % 2 == 0) ^ inverse) {
    ++x;
  }
  return cast<ray_t>(cam, left, top, x, y, max_x, max_y);
}

/// Casts ray-packs where half of the pixels are skipped
template <class ray_t>
static inline std::enable_if_t<ray_t::dim::w != 1 || ray_t::dim::h != 1, ray_t>
sparse_cast(const Camera &cam, const Vector3 &left, const Vector3 &top,
            const typename ray_t::dim_t x, const typename ray_t::dim_t y,
            float max_x, float max_y, bool inverse = false) {
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
  using sparse = sampler::sparse_dim<typename ray_t::dim>;
  auto i = unsigned(0);
  for (unsigned i_y = 0; i_y < sparse::h; ++i_y) {
    float f_y = 1.f - (2 * (y + i_y) + 1) / max_y;
    auto t = top * f_y;
    for (unsigned i_x = 0; i_x < sparse::w; ++i_x) {
      if (((i_x % 2 == 0 && i_y % 2 == 0) || (i_x % 2 == 1 && i_y % 2 == 1)) ^
          inverse) {
        float f_x = 1.f - (2 * (x + i_x) + 1) / max_x;
        auto l = left * f_x;
        X[i] += t.x + l.x;
        Y[i] += t.y + l.y;
        Z[i] += t.z + l.z;
        ++i;
      }
    }
  }
  direction.x = simd::load_ps(X.data());
  direction.y = simd::load_ps(Y.data());
  direction.z = simd::load_ps(Z.data());
  direction.normalize();
  return {origin, direction};
}

template <class ray_t>
static inline std::enable_if_t<ray_t::dim::w == 1 && ray_t::dim::h == 1>
sparse_writeColorToImage(typename ray_t::color_t c, ImageView &image,
                         IntDimension2::dim_t x, IntDimension2::dim_t y,
                         IntDimension2::dim_t global_y, bool inverse = false) {
  if ((global_y % 2 == 1 && x % 2 == 0) ^ inverse) {
    ++x;
  }
  if (x >= image.resolution.w)
    return;
  image.setPixel(x, y, c);
}

template <class ray_t>
static inline std::enable_if_t<ray_t::dim::w != 1 && ray_t::dim::h != 1>
sparse_writeColorToImage(typename ray_t::color_t c, ImageView &image,
                         IntDimension2::dim_t x, IntDimension2::dim_t y,
                         IntDimension2::dim_t global_y, bool inverse = false) {
  using sparse = sampler::sparse_dim<typename ray_t::dim>;
  alignas(simd::REQUIRED_ALIGNMENT)
      std::array<float, simd::REGISTER_CAPACITY_FLOAT>
          R;
  alignas(simd::REQUIRED_ALIGNMENT)
      std::array<float, simd::REGISTER_CAPACITY_FLOAT>
          G;
  alignas(simd::REQUIRED_ALIGNMENT)
      std::array<float, simd::REGISTER_CAPACITY_FLOAT>
          B;
  simd::store_ps(R.data(), c.x);
  simd::store_ps(G.data(), c.y);
  simd::store_ps(B.data(), c.z);
  unsigned idx = 0;
  for (unsigned i_y = 0; i_y < sparse::h && i_y + y < image.resolution.h;
       ++i_y) {
    for (unsigned i_x = 0; i_x < sparse::w && i_x + x < image.resolution.w;
         ++i_x) {
      if (((i_x % 2 == 0 && i_y % 2 == 0) || (i_x % 2 == 1 && i_y % 2 == 1)) ^
          inverse) {
        image.setPixel(x + i_x, y + i_y, {R[idx], G[idx], B[idx]});
        ++idx;
      }
    }
  }
}
}

template <class scene_t, class tracer_t, class ray_t> struct standard_sampler {
  static void render(const scene_t &scene, ImageView &image, tracer_t &tracer) {
    Vector3 left, right, bottom, top;
    const float aspect = (float)image.img.resolution.h / image.img.resolution.w;
    scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

    float max_x = (float)image.img.resolution.w;
    float max_y = (float)image.img.resolution.h;

#ifdef WITH_OMP
#pragma omp parallel for schedule(dynamic, 30) collapse(2)
#endif
    for (long local_y = 0; local_y < image.resolution.h;
         local_y += ray_t::dim::h) {
      for (long x = 0; x < image.resolution.w; x += ray_t::dim::w) {
        auto y = image.getGlobalY(local_y);
        ray_t ray =
            sampler::cast<ray_t>(scene.camera, left, top, x, y, max_x, max_y);
        auto c = tracer.trace(scene, ray);
        writeColorToImage(c, image, x, local_y);
      }
    }
  }
  using dim_t = typename ray_t::dim_t;
};

template <class scene_t, class tracer_t, class ray_t>
struct interpolating_sampler {
  static void render(const scene_t &scene, ImageView &image, tracer_t &tracer) {
    Vector3 left, right, bottom, top;
    const float aspect = (float)image.img.resolution.h / image.img.resolution.w;
    auto w = image.resolution.w, h = image.resolution.h;
    scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);
    float max_x = (float)w;
    float max_y = (float)image.img.resolution.h;
    using sparse = sampler::sparse_dim<typename ray_t::dim>;
#ifdef WITH_OMP
#pragma omp parallel for schedule(dynamic, 30) collapse(2)
#endif
    for (long local_y = 0; local_y < h; local_y += sparse::h) {
      for (long x = 0; x < w; x += sparse::w) {
        auto y = image.getGlobalY(local_y);
        auto ray = sampler::sparse_cast<ray_t>(scene.camera, left, top, x, y,
                                               max_x, max_y);
        auto c = tracer.trace(scene, ray);
        sampler::sparse_writeColorToImage<ray_t>(c, image, x, local_y, y);
      }
    }
#ifdef WITH_OMP
#pragma omp parallel for collapse(2)
#endif
    for (long y = 0; y < h; ++y) {
      for (long x = 0; x < w; ++x) {
        // This is a bit fragile since it relies on the duplicated pattern
        // from sparse_cast (i.e. checkers pattern where 0,0 is the first
        // set pixel
        if ((x % 2 == 0 && y % 2 == 0) || (x % 2 == 1 && y % 2 == 1)) {
          continue;
        } else {
          // TODO maybe it's nicer (and faster) to have 5 different loops
          // for left/top/right/bottom edges plus inner pixels
          Color c{0.f, 0.f, 0.f};
          int n = 0;
          x > 0 ? c += image.getPixel(x - 1, y), ++n : 0;
          y > 0 ? c += image.getPixel(x, y - 1), ++n : 0;
          x < w - 1 ? c += image.getPixel(x + 1, y), ++n : 0;
          y < h - 1 ? c += image.getPixel(x, y + 1), ++n : 0;
          c /= n;
          image.setPixel(x, y, c);
        }
      }
    }
  }
};

template <class scene_t, class tracer_t, class ray_t> struct adaptive_sampler {
private:
  constexpr static float threshold = 0.7;

  static inline float difference(const Color &c1, const Color &c2) {
    auto diff = c1 - c2;
    diff = diff.abs(diff);
    return diff.r + diff.g + diff.b; // Returns sum of distances
  }

  static inline bool error(const typename IntDimension2::dim_t startx,
			   const typename IntDimension2::dim_t starty,
			   const typename IntDimension2::dim_t end_x,
			   const typename IntDimension2::dim_t end_y,
			   ImageView &image) {
    auto w = image.resolution.w, h = image.resolution.h;
    // TODO this uses that rays are cast in a checkered pattern and x is always
	// even
	Color color{0, 0, 0};

	auto regionSizeX = end_x - startx;
	auto regionSizeY = end_y - starty;

	//Make a threshold for pixel comparison
	const int regionSize = 4;

	auto startx2 = startx;
	auto starty2 = starty;
	auto end_x2 = end_x;
	auto end_y2 = end_y;

	if (regionSizeX < regionSize) {
		startx2 = startx - (regionSize - regionSizeX) / 2;
		if (startx2 < 0) startx2 = 0;
		end_x2 = end_x + (regionSize - regionSizeX) / 2;
		if (end_x2 > w) end_x2 = w;
	}

	if (regionSizeY < regionSize) {
		starty2 = starty - (regionSize - regionSizeY) / 2;
		if (starty2 < 0) starty2 = 0;
		end_y2 = end_y + (regionSize - regionSizeY) / 2;
		if (end_y2 > h) end_y2 = h;
	}

	for (long y = starty2; y < end_y2 && y < h; y += 1) {
	  for (long x = startx2; x < end_x2 && x < w; x += 1) {
		color += image.getPixel(x, y);
      }
    }
    color /= ((end_x - startx) * (end_y - starty) / 2);
    for (long y = starty; y < end_y && y < h; y += 1) {
      for (long x = startx; x < end_x && x < w; x += 1) {
	if (!((x % 2 == 0 && y % 2 == 0) || (x % 2 == 1 && y % 2 == 1))) {
	  continue;
	} else if (difference(image.getPixel(x, y), color) > threshold) {
	  return true;
	}
      }
    }
    return false;
  }

  static void interpolate(ImageView &image, long startx, long starty) {
    using sparse = sampler::sparse_dim<typename ray_t::dim>;
    auto w = image.resolution.w, h = image.resolution.h;
    for (long y = starty; y < (starty + sparse::h) && y < h; ++y) {
      auto glob_y = image.getGlobalY(y);
      for (long x = startx; x < (startx + sparse::w) && x < w; ++x) {
	// This is a bit fragile since it relies on the duplicated pattern
	// from sparse_cast (i.e. checkers pattern where 0,0 is the first
	// set pixel
	if ((x % 2 == 0 && glob_y % 2 == 0) || (x % 2 == 1 && glob_y % 2 == 1)) {
	  continue;
	} else {
	  // TODO maybe it's nicer (and faster) to have 5 different loops
	  // for left/top/right/bottom edges plus inner pixels
	  Color c{0.f, 0.f, 0.f};
	  int n = 0;
	  x > 0 ? c += image.getPixel(x - 1, y), ++n : 0;
	  y > 0 ? c += image.getPixel(x, y - 1), ++n : 0;
	  x < w - 1 ? c += image.getPixel(x + 1, y), ++n : 0;
	  y < h - 1 ? c += image.getPixel(x, y + 1), ++n : 0;
	  c /= n;
	  image.setPixel(x, y, c);
	}
      }
    }
  }
  
public:
  static void render(const scene_t &scene, ImageView &image, tracer_t &tracer) {
    Vector3 left, right, bottom, top;
    const float aspect = (float)image.img.resolution.h / image.img.resolution.w;
    auto w = image.resolution.w, h = image.resolution.h;
    scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

    float max_x = (float)w;
    float max_y = (float)image.img.resolution.h;

    using sparse = sampler::sparse_dim<typename ray_t::dim>;

#ifdef WITH_OMP
#pragma omp parallel for schedule(dynamic, 30) collapse(2)
#endif
    for (long local_y = 0; local_y < h; local_y += sparse::h) {
      for (long x = 0; x < w; x += sparse::w) {
        auto y = image.getGlobalY(local_y);
        auto ray = sampler::sparse_cast<ray_t>(scene.camera, left, top, x, y,
                                               max_x, max_y);
        auto c = tracer.trace(scene, ray);
        sampler::sparse_writeColorToImage<ray_t>(c, image, x, local_y, y);
      }
    }
#ifdef WITH_OMP
#pragma omp parallel for schedule(dynamic, 30) collapse(2)
#endif
    for (long local_y = 0; local_y < h; local_y += sparse::h) {
      for (long x = 0; x < w; x += sparse::w) {
        auto y = image.getGlobalY(local_y);
		if (!error(x, y, x + sparse::w, y + sparse::h, image)) {
          interpolate(image, x, local_y);
        } else {
          auto ray_inverse = sampler::sparse_cast<ray_t>(
              scene.camera, left, top, x, y, max_x, max_y, true);
          auto c_inverse = tracer.trace(scene, ray_inverse);
          sampler::sparse_writeColorToImage<ray_t>(c_inverse, image, x, local_y,
                                                   y, true);
        }
      }
    }
  }
};

#endif // PRAY_SAMPLER_HPP
