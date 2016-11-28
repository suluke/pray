#ifndef PRAY_H
#define PRAY_H
/* Dependency: image writing to disk: PNG, TGA, BMP
*  https://github.com/nothings/stb
*/
#include "stb_image_write.h"

/* Dependency: JSON for Modern C++
*  https://nlohmann.github.io/json/
*/
#include "json.hpp"
using json = nlohmann::json;

/* Dependency: Tiny but powerful single file wavefront obj loader
*  https://github.com/syoyo/tinyobjloader
*/
#include "tiny_obj_loader.h"

namespace pray {
template <class T> struct Vec3 {
  Vec3() = default;
  Vec3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}
  Vec3(const T v[3]) : x(v[0]), y(v[1]), z(v[2]) {}

  /* ... */

  T x;
  T y;
  T z;
};
}

#endif /* PRAY_H */
