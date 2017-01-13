#ifndef PRAY_SIMD_DEBUG_HPP
#define PRAY_SIMD_DEBUG_HPP

#define _CONCAT2(s1, s2) s1 ## s2
#define _CONCAT3(s1, s2, s3) s1 ## s2 ## s3
#define _CONCAT4(s1, s2, s3, s4) s1 ## s2 ## s3 ## s4
#define CONCAT2(s1, s2) _CONCAT2(s1, s2)
#define CONCAT3(s1, s2, s3) _CONCAT3(s1, s2, s3)
#define CONCAT4(s1, s2, s3, s4) _CONCAT4(s1, s2, s3, s4)

#if defined(__AVX__)
#include <immintrin.h>

#define MACRO_REGISTER_SIZE_BYTES 32u
#define MACRO_REGISTER_SIZE_BITS 256
#define MACRO_INTRIN_PREFIX _mm256_

namespace simd {
  // comparisons
  static inline __m256 cmplt_ps(__m256 a, __m256 b) {
    return _mm256_cmp_ps(a, b, _CMP_LT_OS);
  }
  static inline __m256 cmple_ps(__m256 a, __m256 b) {
    return _mm256_cmp_ps(a, b, _CMP_LT_OS);
  }
  static inline __m256i cmpeq_epi32(__m256i a, __m256i b) {
    return _mm256_castps_si256(_mm256_cmp_ps(_mm256_xor_ps(_mm256_castsi256_ps(a), _mm256_castsi256_ps(b)), _mm256_setzero_ps(), _CMP_EQ_OS));
  }

  // http://stackoverflow.com/a/16018927/1468532
  static inline bool isAll(__m256i b) {
    return _mm256_movemask_ps(_mm256_castsi256_ps(b)) == 0xff;
  }
  static inline bool isAny(__m256i b) {
    return _mm256_movemask_ps(_mm256_castsi256_ps(b)) != 0x0;
  }
}

#elif defined(__SSE2__)

#include <emmintrin.h>

#define MACRO_REGISTER_SIZE_BYTES 16u
#define MACRO_REGISTER_SIZE_BITS 128
#define MACRO_INTRIN_PREFIX _mm_

namespace simd {
  // comparisons
  constexpr auto cmplt_ps = _mm_cmplt_ps;
  constexpr auto cmple_ps = _mm_cmple_ps;
  constexpr auto cmpeq_epi32 = _mm_cmpeq_epi32;

  static inline bool isAll(__m128i b) {
    return _mm_movemask_epi8(b) == 0xffff;
  }
  static inline bool isAny(__m128i b) {
    return _mm_movemask_epi8(b) != 0x0;
  }
}

#else

#error instruction set not supported

#endif

namespace simd {
  static constexpr auto REGISTER_SIZE_BYTES = MACRO_REGISTER_SIZE_BYTES;
  static constexpr auto REGISTER_CAPACITY_FLOAT = (REGISTER_SIZE_BYTES/sizeof(float));
  static constexpr auto REGISTER_CAPACITY_I32 = (REGISTER_SIZE_BYTES/sizeof(uint32_t));

  using floatty = CONCAT2(__m, MACRO_REGISTER_SIZE_BITS);
  using intty = CONCAT3(__m, MACRO_REGISTER_SIZE_BITS, i);

  // casts
  static inline intty   castps_si(floatty arg) { return CONCAT3(MACRO_INTRIN_PREFIX, castps_si, MACRO_REGISTER_SIZE_BITS) (arg); }
  static inline floatty castsi_ps(intty   arg) { return CONCAT4(MACRO_INTRIN_PREFIX, castsi, MACRO_REGISTER_SIZE_BITS, _ps) (arg); }

  // binary
  static inline floatty and_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, and_ps)(f1, f2); }
  static inline floatty or_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, or_ps)(f1, f2); }
  static inline floatty xor_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, xor_ps)(f1, f2); }
  static inline floatty not_ps (floatty x) {
    return CONCAT2(MACRO_INTRIN_PREFIX, xor_ps)(x, CONCAT4(MACRO_INTRIN_PREFIX, castsi, MACRO_REGISTER_SIZE_BITS, _ps)(CONCAT2(MACRO_INTRIN_PREFIX, set1_epi32(-1))));
  }

  // basic arithmetic
  static inline floatty add_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, add_ps)(f1, f2); }
  static inline floatty sub_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, sub_ps)(f1, f2); }
  static inline floatty mul_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, mul_ps)(f1, f2); }
  static inline floatty div_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, div_ps)(f1, f2); }

  // advanced arithmetic
  static inline floatty min_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, min_ps)(f1, f2); }
  static inline floatty max_ps(floatty f1, floatty f2) { return CONCAT2(MACRO_INTRIN_PREFIX, max_ps)(f1, f2); }
  static inline floatty sqrt_ps(floatty f) { return CONCAT2(MACRO_INTRIN_PREFIX, sqrt_ps)(f); }

  // setting
  static inline floatty set1_ps(float f) { return CONCAT2(MACRO_INTRIN_PREFIX, set1_ps)(f); }
  static inline intty set1_epi32(int i) { return CONCAT2(MACRO_INTRIN_PREFIX, set1_epi32)(i); }
  static inline floatty setzero_ps() { return CONCAT2(MACRO_INTRIN_PREFIX, setzero_ps)(); }

  // memory
  static inline floatty load_ps(const float *addr) { return CONCAT2(MACRO_INTRIN_PREFIX, load_ps)(addr); }
  static inline void store_ps(float *addr, floatty val) { CONCAT2(MACRO_INTRIN_PREFIX, store_ps)(addr, val); }
  static inline void store_si(intty *addr, intty val) { CONCAT3(MACRO_INTRIN_PREFIX, store_si, MACRO_REGISTER_SIZE_BITS)(addr, val); }
}
#endif // PRAY_SIMD_DEBUG_HPP
