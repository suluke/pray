#pragma once

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
