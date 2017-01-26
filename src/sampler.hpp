#ifndef PRAY_SAMPLER_HPP
#define PRAY_SAMPLER_HPP
#pragma once

namespace sampler {
  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w == 1 && ray_t::dim::h == 1, ray_t>
      cast(const Camera &cam, const Vector3 &left, const Vector3 &top,
           const typename ray_t::dim_t x, const typename ray_t::dim_t y, float max_x, float max_y)
  {
    auto origin = cam.position;
    auto direction = Vector3((top * (1.f - (2 * y + 1) / max_y) + left * (1.f - (2 * x + 1) / max_x) + cam.direction).normalize());
    return {origin, direction};
  }
  
  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w != 1 || ray_t::dim::h != 1, ray_t>
      cast(const Camera &cam, const Vector3 &left, const Vector3 &top,
           const typename ray_t::dim_t x, const typename ray_t::dim_t y, float max_x, float max_y)
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

  /// If cast in sparse mode, ray dimensions change
  template<typename dim>
  struct sparse_dim {
    static const unsigned w = (dim::w <= dim::h ? dim::w * 2 : dim::w);
    static const unsigned h = (dim::h <  dim::w ? dim::h * 2 : dim::h);
  };

  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w == 1 && ray_t::dim::h == 1, ray_t>
      sparse_cast(const Camera &cam, const Vector3 &left, const Vector3 &top,
                  typename ray_t::dim_t x, const typename ray_t::dim_t y,
                  float max_x, float max_y, bool inverse = false)
  {
    // For odd rows, we cast the ray for the right of two neighboring pixels 
    if ((y % 2 == 1 && x % 2 == 0) ^ inverse) {
      ++x;
    }
    return cast<ray_t>(cam, left, top, x, y, max_x, max_y);
  }

  /// Casts ray-packs where half of the pixels are skipped
  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w != 1 || ray_t::dim::h != 1, ray_t>
      sparse_cast(const Camera &cam, const Vector3 &left, const Vector3 &top,
                  const typename ray_t::dim_t x, const typename ray_t::dim_t y,
                  float max_x, float max_y, bool inverse = false)
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
    using sparse = sampler::sparse_dim<typename ray_t::dim>;
    auto i = unsigned(0);
    for (unsigned i_y = 0; i_y < sparse::h; ++i_y) {
      float f_y = 1.f - (2 * (y + i_y) + 1) / max_y;
      auto t = top * f_y;
      for (unsigned i_x = 0; i_x < sparse::w; ++i_x) {
        if (((i_x % 2 == 0 && i_y % 2 == 0) || (i_x % 2 == 1 && i_y % 2 == 1)) ^ inverse) {
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

  float difference(const Color& c1,const Color& c2) {
	  //std::cout << "Debug: " << " Color1(" << c1.r << "," << c1.g << "," << c1.b << ")" << " Color2(" << c2.r << "," << c2.g << "," << c2.b << ")\n";
	  auto square = c1-c2;
	  square = square * square;
	  return square.r + square.g + square.b; //Returns squared distance
  }


  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w != 1 || ray_t::dim::h != 1, bool>
  error(const typename ray_t::dim_t x, const typename ray_t::dim_t y, const typename ray_t::dim_t end_x, const typename ray_t::dim_t end_y, ImageView &image) {
	  //TODO this uses that rays are cast in a checkered pattern and x is always even
	  const float threshold = 2.f;
	  Color color{};
	  for (long i=x;i<end_x;i+=1) {
		  for (long j=y;j<end_y;j+=1) {
			  color += image.getPixel(i,j);
		  }
	  }
	  color /= ((end_x - x) * (end_y - y) / 2);
	  for (long i=x;i<end_x;i+=1) {
		  for (long j=y;j<end_y;j+=1) {
			  if (!((i%2 == 0 && j%2 == 0) || (i%2 == 1 && j%2 == 1))) {
				  continue;
			  } else if (sampler::difference(image.getPixel(i,j),color)>threshold) {
				  return true;
			  }
		  }
	  }
	  return false;
  }

  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w == 1 || ray_t::dim::h == 1, bool>
  error(const typename ray_t::dim_t x, const typename ray_t::dim_t y, const typename ray_t::dim_t end_x, const typename ray_t::dim_t end_y, ImageView &image) {
	  //TODO this uses that rays are cast in a checkered pattern and x is always even
	  const float threshold = 2.f;
	  Color color{};
	  for (long i=x;i<end_x;i+=1) {
		  for (long j=y;j<end_y;j+=1) {
			  color += image.getPixel(i,j);
		  }
	  }
	  color /= ((end_x - x) * (end_y - y) / 2);
	  for (long i=x;i<end_x;i+=1) {
		  for (long j=y;j<end_y;j+=1) {
			  if (!((i%2 == 0 && j%2 == 0) || (i%2 == 1 && j%2 == 1))) {
				  continue;
			  } else if (sampler::difference(image.getPixel(i,j),color)>threshold) {
				  return true;
			  }
		  }
	  }
	  return false;
  }

  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w == 1 && ray_t::dim::h == 1>
      sparse_writeColorToImage(typename ray_t::color_t c, ImageView &image,
                               IntDimension2::dim_t x, IntDimension2::dim_t y, IntDimension2::dim_t global_y, bool inverse = false)
  {
    if ((global_y % 2 == 1 && x % 2 == 0) ^ inverse) {
      ++x;
    }
    image.setPixel(x, y, c);
  }
  
  template<class ray_t>
  static inline std::enable_if_t<ray_t::dim::w != 1 && ray_t::dim::h != 1>
      sparse_writeColorToImage(typename ray_t::color_t c, ImageView &image,
                               IntDimension2::dim_t x, IntDimension2::dim_t y, IntDimension2::dim_t global_y, bool inverse = false)
  {
    using sparse = sampler::sparse_dim<typename ray_t::dim>;
    alignas(simd::REQUIRED_ALIGNMENT) std::array<float, simd::REGISTER_CAPACITY_FLOAT> R;
    alignas(simd::REQUIRED_ALIGNMENT) std::array<float, simd::REGISTER_CAPACITY_FLOAT> G;
    alignas(simd::REQUIRED_ALIGNMENT) std::array<float, simd::REGISTER_CAPACITY_FLOAT> B;
    simd::store_ps(R.data(), c.x);
    simd::store_ps(G.data(), c.y);
    simd::store_ps(B.data(), c.z);
    unsigned idx = 0;
    for (unsigned i_y = 0; i_y < sparse::h && i_y + y < image.resolution.h; ++i_y) {
      for (unsigned i_x = 0; i_x < sparse::w && i_x + x < image.resolution.w; ++i_x) {
        if (((i_x % 2 == 0 && i_y % 2 == 0) || (i_x % 2 == 1 && i_y % 2 == 1)) ^ inverse) {
          image.setPixel(x + i_x, y + i_y, {R[idx], G[idx], B[idx]});
          ++idx;
        }
      }
    }
  }
}

template<class scene_t, class tracer_t, class ray_t>
struct standard_sampler {
  static void render(const scene_t &scene, ImageView &image, tracer_t &tracer) {
    Vector3 left, right, bottom, top;
    const float aspect = (float) image.resolution.h / image.resolution.w;
    scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

    float max_x = (float) image.resolution.w;
    float max_y = (float) image.img.resolution.h;

#ifdef WITH_OMP
	#pragma omp parallel for schedule(dynamic, 30) collapse(2)
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
	auto w = image.resolution.w, h = image.resolution.h;
	scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);
	float max_x = (float) w;
	float max_y = (float) h;
	using sparse = sampler::sparse_dim<typename ray_t::dim>;
#ifdef WITH_OMP
	#pragma omp parallel for schedule(dynamic, 30) collapse(2)
#endif
	for (long local_y = 0; local_y < h; local_y += sparse::h) {
	  for (long x = 0; x < w; x += sparse::w) {
		auto y = image.getGlobalY(local_y);
		auto ray = sampler::sparse_cast<ray_t>(scene.camera, left, top, x, y, max_x, max_y);
		auto c = tracer.trace(scene, ray);
		sampler::sparse_writeColorToImage<ray_t>(c, image, x, local_y, y);
	  }
	}
#ifdef WITH_OMP
	#pragma omp parallel for collapse(2)
#endif
	for(long y = 0; y < h; ++y) {
	  for(long x = 0; x < w; ++x) {
		// This is a bit fragile since it relies on the duplicated pattern
		// from sparse_cast (i.e. checkers pattern where 0,0 is the first
		// set pixel
		if ((x%2 == 0 && y%2 == 0) || (x%2 == 1 && y%2 == 1)) {
		  continue;
		} else {
		  // TODO maybe it's nicer (and faster) to have 5 different loops
		  // for left/top/right/bottom edges plus inner pixels
		  Color c{0.f, 0.f, 0.f};
		  int n = 0;
		  x > 0   ? c += image.getPixel(x-1,y), ++n : 0;
		  y > 1   ? c += image.getPixel(x,y-1), ++n : 0;
		  x < w-1 ? c += image.getPixel(x+1,y), ++n : 0;
		  y < h-1 ? c += image.getPixel(x,y+1), ++n : 0;
		  c /= n;
		  image.setPixel(x, y, c);
		}
	  }
	}
  }
};

template<class scene_t, class tracer_t, class ray_t>
struct adaptive_sampler {

  static void render(const scene_t &scene, ImageView &image, tracer_t &tracer) {
    Vector3 left, right, bottom, top;
    const float aspect = (float) image.resolution.h / image.resolution.w;
    auto w = image.resolution.w, h = image.resolution.h;
    scene.camera.calculateFrustumVectors(aspect, &left, &right, &bottom, &top);

    float max_x = (float) w;
    float max_y = (float) h;

    using sparse = sampler::sparse_dim<typename ray_t::dim>;

#ifdef WITH_OMP
	#pragma omp parallel for schedule(dynamic, 30) collapse(2)
#endif
    for (long local_y = 0; local_y < h; local_y += sparse::h) {
      for (long x = 0; x < w; x += sparse::w) {
        auto y = image.getGlobalY(local_y);
        auto ray = sampler::sparse_cast<ray_t>(scene.camera, left, top, x, y, max_x, max_y);
		auto c = tracer.trace(scene, ray);
        sampler::sparse_writeColorToImage<ray_t>(c, image, x, local_y, y);
      }
    }
#ifdef WITH_OMP
    #pragma omp parallel for collapse(2)
#endif
	for(long local_y = 0; local_y < h; local_y += sparse::h) {
	  for(long x = 0; x < w; x += sparse::w) {
		auto y = image.getGlobalY(local_y);
		if (!sampler::error<ray_t>(x,y,x+sparse::w,y+sparse::h,image)) {
			for (long i = y; i < (y+sparse::h) && i < h; ++i) {
				for (long j = x; j < (x+sparse::w) && j < w; ++j) {
					// This is a bit fragile since it relies on the duplicated pattern
					// from sparse_cast (i.e. checkers pattern where 0,0 is the first
					// set pixel
					if ((j%2 == 0 && i%2 == 0) || (j%2 == 1 && i%2 == 1)) {
					  continue;
					} else {
					  // TODO maybe it's nicer (and faster) to have 5 different loops
					  // for left/top/right/bottom edges plus inner pixels
					  Color c{0.f, 0.f, 0.f};
					  int n = 0;
					  j > 0   ? c += image.getPixel(j-1,i), ++n : 0;
					  i > 0   ? c += image.getPixel(j,i-1), ++n : 0;
					  j < w-1 ? c += image.getPixel(j+1,i), ++n : 0;
					  i < h-1 ? c += image.getPixel(j,i+1), ++n : 0;
					  c /= n;
					  image.setPixel(j, i, c);
					}
				}
			}
        } else {
			auto ray_inverse = sampler::sparse_cast<ray_t>(scene.camera, left, top, x, y, max_x, max_y,true);
			auto c_inverse = tracer.trace(scene,ray_inverse);
			sampler::sparse_writeColorToImage<ray_t>(c_inverse,image,x,local_y,y,true);
        }
	  }
	}
  }
};

#endif // PRAY_SAMPLER_HPP
