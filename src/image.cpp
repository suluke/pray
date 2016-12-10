#include "image.hpp"
#include "ext.hpp"

bool Image::save(const std::string &filename) const {
  int write_error = stbi_write_bmp(filename.c_str(), resolution.w, resolution.h, 3, pixels.data());
  if(write_error == 0)
  {
    std::cerr << "could not write output image" << "\n";
    return false;
  }
  return true;
}
