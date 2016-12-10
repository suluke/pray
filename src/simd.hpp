#if defined(__AVX__)

#include <immintrin.h>

namespace simd {
  constexpr auto REGISTER_SIZE_BYTES =  32u;
  constexpr auto REGISTER_CAPACITY_INT8 = (REGISTER_SIZE_BYTES/sizeof(int8_t));

  using f_t = __m256;
}

#elif defined(__SSE3__)

#include <ammintrin.h>
namespace simd {
  constexpr auto REGISTER_SIZE_BYTES = 16u;
  constexpr auto REGISTER_CAPACITY_INT8 = (REGISTER_SIZE_BYTES/sizeof(int8_t));

  using f_t = __m128;
}

#else

#error instruction set not supported

#endif
