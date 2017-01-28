#ifndef PRAY_SSE_MATH_HPP
#define PRAY_SSE_MATH_HPP

#include <pmmintrin.h>

inline __m128 set_vector3_ps(const Vector3 &v) {
    return _mm_set_ps(0.f, v.z, v.y, v.x);
}

// make sure that either a[3] or b[3] == 0.f, otherwise this is wrong!!!
inline float fast_dot_ps(__m128 a, __m128 b) {
    const __m128 t = _mm_mul_ps(a, b);
    const __m128 r = _mm_hadd_ps(_mm_hadd_ps(t, t), t); // sse3...

    // this is optimized to a no-op
    float r_f;
    _mm_store_ss(&r_f, r);
    return r_f;
}

inline __m128 cross_ps(__m128 a, __m128 b) {
    /*
    shema:
    z = (a1*b2 - a2*b1)
    x = (a2*b3 - a3*b2)
    y = (a3*b1 - a1*b3)
    w = trash
    "Rotating" coordinates around by 1 we safe one _mm_shuffle_ps in summary because we can use operands a and b as colums 1 and 4 directly.
    */

    // cloumns
    const __m128 c1 = a;
    const __m128 c2 = _mm_shuffle_ps(b, b, 0b001001);
    const __m128 c3 = _mm_shuffle_ps(a, a, 0b001001);
    const __m128 c4 = b;

    // multiplications
    const __m128 m1 = _mm_mul_ps(c1, c2);
    const __m128 m2 = _mm_mul_ps(c3, c4);

    // subtraction
    const __m128 r = _mm_sub_ps(m1, m2);

    // shuffle result so that we return the correct in-register layout
    return _mm_shuffle_ps(r, r, 0b001001);
}

#endif // PRAY_SSE_MATH_HPP
