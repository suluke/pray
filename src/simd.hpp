#include <array>

#ifdef DEBUG

#include "simd_debug.hpp"

#else
#if defined(__AVX__)

#include <immintrin.h>
#include "math.hpp"

namespace simd {
  static constexpr auto REGISTER_SIZE_BYTES = 32u;
  static constexpr auto REGISTER_CAPACITY_FLOAT = (REGISTER_SIZE_BYTES/sizeof(float));
  static constexpr auto REGISTER_CAPACITY_I32 = (REGISTER_SIZE_BYTES/sizeof(uint32_t));

  using floatty = __m256;
  using intty = __m256i;

  // casts
  static constexpr auto castps_si = _mm256_castps_si256;
  static constexpr auto castsi_ps = _mm256_castsi256_ps;

  // binary
  static constexpr auto and_ps = _mm256_and_ps;
  static constexpr auto or_ps = _mm256_or_ps;
  static constexpr auto xor_ps = _mm256_xor_ps;
  static inline floatty not_ps (floatty x) {
    return _mm256_xor_ps(x, _mm256_castsi256_ps(_mm256_set1_epi32(-1)));
  }

  // basic arithmetic
  static constexpr auto add_ps = _mm256_add_ps;
  static constexpr auto sub_ps = _mm256_sub_ps;
  static constexpr auto mul_ps = _mm256_mul_ps;
  static constexpr auto div_ps = _mm256_div_ps;

  // advanced arithmetic
  static constexpr auto min_ps = _mm256_min_ps;
  static constexpr auto max_ps = _mm256_max_ps;
  static constexpr auto sqrt_ps = _mm256_sqrt_ps;

  // comparisons
  static inline floatty cmplt_ps(floatty a, floatty b) {
    return _mm256_cmp_ps(a, b, _CMP_LT_OS);
  }
  static inline floatty cmple_ps(floatty a, floatty b) {
    return _mm256_cmp_ps(a, b, _CMP_LT_OS);
  }
  static inline intty cmpeq_epi32(intty a, intty b) {
    return castps_si(_mm256_cmp_ps(xor_ps(castsi_ps(a), castsi_ps(b)), _mm256_setzero_ps(), _CMP_EQ_OS));
  }

  // http://stackoverflow.com/a/16018927/1468532
  static inline bool isAll(intty b) {
    return _mm256_movemask_ps(simd::castsi_ps(b)) == 0xff;
  }
  static inline bool isAny(intty b) {
    return _mm256_movemask_ps(simd::castsi_ps(b)) != 0x0;
  }

  // setting
  static constexpr auto set1_ps = _mm256_set1_ps;
  static constexpr auto set1_epi32 = _mm256_set1_epi32;
  static constexpr auto setzero_ps = _mm256_setzero_ps;

  // memory
  static constexpr auto load_ps = _mm256_load_ps;
  static constexpr auto store_ps = _mm256_store_ps;
  static constexpr auto store_si = _mm256_store_si256;
}

#elif defined(__SSE2__)

#include <emmintrin.h>

namespace simd {
  static constexpr auto REGISTER_SIZE_BYTES = 16u;
  static constexpr auto REGISTER_CAPACITY_FLOAT = (REGISTER_SIZE_BYTES/sizeof(float));
  static constexpr auto REGISTER_CAPACITY_I32 = (REGISTER_SIZE_BYTES/sizeof(uint32_t));

  using floatty = __m128;
  using intty = __m128i;

  // casts
  static constexpr auto castps_si = _mm_castps_si128;
  static constexpr auto castsi_ps = _mm_castsi128_ps;

  // binary
  static constexpr auto and_ps = _mm_and_ps;
  static constexpr auto or_ps = _mm_or_ps;
  static constexpr auto xor_ps = _mm_xor_ps;
  static inline floatty not_ps (floatty x) {
    return _mm_xor_ps(x, _mm_castsi128_ps(_mm_set1_epi32(-1)));
  }

  // basic arithmetic
  static constexpr auto add_ps = _mm_add_ps;
  static constexpr auto sub_ps = _mm_sub_ps;
  static constexpr auto mul_ps = _mm_mul_ps;
  static constexpr auto div_ps = _mm_div_ps;

  // advanced arithmetic
  static constexpr auto min_ps = _mm_min_ps;
  static constexpr auto max_ps = _mm_max_ps;
  static constexpr auto sqrt_ps = _mm_sqrt_ps;

  // comparisons
  static constexpr auto cmplt_ps = _mm_cmplt_ps;
  static constexpr auto cmple_ps = _mm_cmple_ps;
  static constexpr auto cmpeq_epi32 = _mm_cmpeq_epi32;
  
  static inline bool isAll(intty b) {
    return _mm_movemask_epi8(b) == 0xffff;
  }
  static inline bool isAny(intty b) {
    return _mm_movemask_epi8(b) != 0x0;
  }

  // setting
  static constexpr auto set1_ps = _mm_set1_ps;
  static constexpr auto set1_epi32 = _mm_set1_epi32;
  static constexpr auto setzero_ps = _mm_setzero_ps;

  // memory
  static constexpr auto load_ps = _mm_load_ps;
  static constexpr auto store_ps = _mm_store_ps;
  static constexpr auto store_si = _mm_store_si128;
}

#else

#error instruction set not supported

#endif // __AVX__ / __SSE__
#endif // DEBUG

namespace simd {
  static inline floatty abs_ps(floatty f) {
    return and_ps(f, castsi_ps(set1_epi32(0x7fffffff)));
  }
}

namespace simd {
  struct Vec3Pack {
    using component_t = floatty;
    
    component_t x, y, z;

    Vec3Pack() {}
    Vec3Pack(component_t x, component_t y, component_t z) : x(x), y(y), z(z) {}
    Vec3Pack(Vector3 v) : x(set1_ps(v.x)), y(set1_ps(v.y)), z(set1_ps(v.z)) {}
    Vec3Pack(float x, float y, float z) : x(set1_ps(x)), y(set1_ps(y)), z(set1_ps(z)) {}
    
    template<class T> const component_t &operator[](T index) const {
      ASSERT(index < 3);
      
      switch (index)
      {
        case 0: return x; break;
        case 1: return y; break;
        case 2: return z; break;
      }
      std::abort();
    }
    template<class T> component_t &operator[](T index) {
      return const_cast<component_t &>(static_cast<const Vec3Pack &>(*this)[index]);
    }
    
    Vec3Pack operator-() const {
      const auto SIGN_MASK = set1_ps(-0.0f);
      return Vec3Pack(xor_ps(x, SIGN_MASK), xor_ps(y, SIGN_MASK), xor_ps(z, SIGN_MASK));
    }
    Vec3Pack operator+(const Vec3Pack &a) const {
      return Vec3Pack(add_ps(x, a.x), add_ps(y, a.y), add_ps(z, a.z));
    }
    Vec3Pack operator-(const Vec3Pack &a) const {
      return Vec3Pack(sub_ps(x, a.x), sub_ps(y, a.y), sub_ps(z, a.z));
    }
    Vec3Pack operator*(const Vec3Pack &a) const {
      return Vec3Pack(mul_ps(x, a.x), mul_ps(y, a.y), mul_ps(z, a.z));
    }
    Vec3Pack operator*(component_t a) const {
      return Vec3Pack(mul_ps(x, a), mul_ps(y, a), mul_ps(z, a));
    }
    Vec3Pack operator/(component_t a) const {
      return Vec3Pack(div_ps(x, a), div_ps(y, a), div_ps(z, a));
    }
    Vec3Pack operator/(Vec3Pack v) const {
      return Vec3Pack(div_ps(x, v.x), div_ps(y, v.y), div_ps(z, v.z));
    }

    Vec3Pack &operator+=(const Vec3Pack &a) { return *this = *this + a; }
    Vec3Pack &operator*=(component_t a) { return *this = *this * a; }
    Vec3Pack &operator/=(component_t a) { return *this = *this / a; }

    component_t dot(const Vec3Pack &a) const {
      const auto X = mul_ps(x, a.x);
      const auto Y = mul_ps(y, a.y);
      const auto Z = mul_ps(z, a.z);
      return add_ps(X, add_ps(Y, Z));
    }
    Vec3Pack cross(const Vec3Pack &a) const {
      const auto X = sub_ps(mul_ps(y, a.z), mul_ps(z, a.y));
      const auto Y = sub_ps(mul_ps(z, a.x), mul_ps(x, a.z));
      const auto Z = sub_ps(mul_ps(x, a.y), mul_ps(y, a.x));
      return Vec3Pack(X, Y, Z);
    }

    component_t lengthSquared() const {
      return dot(*this);
    }
    component_t length() const { return sqrt_ps(lengthSquared()); }
    Vec3Pack &normalize() { /*ASSERT(length() != approx(0));*/ return *this /= length(); }
  };
}

inline std::ostream &operator<<(std::ostream &o, const simd::floatty &f) {
  alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> F;
  simd::store_ps(&F[0], f);
  for (unsigned i = 0; i < simd::REGISTER_CAPACITY_FLOAT; ++i) {
    o << F[i] << ", ";
  }
  return o;
}
inline std::ostream &operator<<(std::ostream &o, const simd::Vec3Pack &v) {
  alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> X;
  alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> Y;
  alignas(16) std::array<float, simd::REGISTER_CAPACITY_FLOAT> Z;
  simd::store_ps(&X[0], v.x);
  simd::store_ps(&Y[0], v.y);
  simd::store_ps(&Z[0], v.z);
  for (unsigned i = 0; i < simd::REGISTER_CAPACITY_FLOAT; ++i) {
    o << "(" << X[i] << ", " << Y[i] << ", " << Z[i] << ")";
  }
  return o;
}
