#if defined(__AVX__)

#include <immintrin.h>

namespace simd {
  static constexpr auto REGISTER_SIZE_BYTES = 32u;
  static constexpr auto REGISTER_CAPACITY_FLOAT = (REGISTER_SIZE_BYTES/sizeof(float));
  static constexpr auto REGISTER_CAPACITY_I32 = (REGISTER_SIZE_BYTES/sizeof(uint32_t));

  using floatty = __m256;
  using intty = __m256i;

  constexpr auto add_ps = _mm256_add_ps;
  constexpr auto sub_ps = _mm256_sub_ps;
  constexpr auto mul_ps = _mm256_mul_ps;
  constexpr auto div_ps = _mm256_div_ps;
  constexpr auto and_ps = _mm256_and_ps;
  constexpr auto xor_ps = _mm256_xor_ps;
  constexpr auto set1_ps = _mm256_set1_ps;
  constexpr auto sqrt_ps = _mm256_sqrt_ps;
  constexpr auto set_ps = _mm256_set_ps;
  constexpr auto load_ps = _mm256_load_ps;
  constexpr auto store_ps = _mm256_store_ps;
  constexpr auto store_si = _mm256_store_si256;
  inline floatty cmplt_ps(floatty a, floatty b) {
    return _mm256_cmp_ps(a, b, _CMP_LT_OS);
  }
  constexpr auto max_ps = _mm256_max_ps;
}

#elif defined(__SSE2__)

#include <emmintrin.h>

namespace simd {
  static constexpr auto REGISTER_SIZE_BYTES = 16u;
  static constexpr auto REGISTER_CAPACITY_FLOAT = (REGISTER_SIZE_BYTES/sizeof(float));
  static constexpr auto REGISTER_CAPACITY_I32 = (REGISTER_SIZE_BYTES/sizeof(uint32_t));

  using floatty = __m128;
  using intty = __m128i;

  constexpr auto add_ps = _mm_add_ps;
  constexpr auto sub_ps = _mm_sub_ps;
  constexpr auto mul_ps = _mm_mul_ps;
  constexpr auto div_ps = _mm_div_ps;
  constexpr auto and_ps = _mm_and_ps;
  constexpr auto xor_ps = _mm_xor_ps;
  constexpr auto set1_ps = _mm_set1_ps;
  constexpr auto sqrt_ps = _mm_sqrt_ps;
  constexpr auto set_ps = _mm_set_ps;
  constexpr auto load_ps = _mm_load_ps;
  constexpr auto store_ps = _mm_store_ps;
  constexpr auto store_si = _mm_store_si128;
  constexpr auto cmplt_ps = _mm_cmplt_ps;
  constexpr auto max_ps = _mm_max_ps;
}

#else

#error instruction set not supported

#endif

namespace simd {
  struct Vec3Pack {
    floatty x, y, z;

    Vec3Pack() {}
    Vec3Pack(floatty x, floatty y, floatty z) : x(x), y(y), z(z) {}
    Vec3Pack(Vector3 v) : x(set1_ps(v.x)), y(set1_ps(v.y)), z(set1_ps(v.z)) {}
    Vec3Pack(float x, float y, float z) : x(set1_ps(x)), y(set1_ps(y)), z(set1_ps(z)) {}

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
    Vec3Pack operator*(floatty a) const {
      return Vec3Pack(mul_ps(x, a), mul_ps(y, a), mul_ps(z, a));
    }
    Vec3Pack operator/(floatty a) const {
      return Vec3Pack(div_ps(x, a), div_ps(y, a), div_ps(z, a));
    }

    Vec3Pack &operator+=(const Vec3Pack &a) { return *this = *this + a; }
    Vec3Pack &operator*=(floatty a) { return *this = *this * a; }
    Vec3Pack &operator/=(floatty a) { return *this = *this / a; }

    floatty dot(const Vec3Pack &a) const {
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

    floatty lengthSquared() const {
      return dot(*this);
    }
    floatty length() const { return sqrt_ps(lengthSquared()); }
    Vec3Pack &normalize() { /*ASSERT(length() != approx(0));*/ return *this /= length(); }
  };
}
