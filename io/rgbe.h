#pragma once

#include <cmath>
#include <fstream>
#include "image.h"
#include "radiometry/rgb.h"

namespace amber {
namespace io {

template <typename RealType>
void export_rgbe(const char* filename, const Image<radiometry::RGB<RealType>>& image)
{
  std::ofstream ofs(filename, std::ofstream::trunc);

  ofs << "#?RADIANCE" << std::endl;
  ofs << "FORMAT=32-bit_rle_rgbe" << std::endl << std::endl;
  ofs << "-Y " << image.m_height << " +X " << image.m_width << std::endl;

  for (size_t j = 0; j < image.m_height; j++) {
    for (size_t i = 0; i < image.m_width; i++) {
      const auto& p = image.pixel(i, j);

      int exponent;
      const auto significand = std::frexp(p.max(), &exponent) * 256 / p.max();

      ofs << static_cast<unsigned char>(significand * p.r());
      ofs << static_cast<unsigned char>(significand * p.g());
      ofs << static_cast<unsigned char>(significand * p.b());
      ofs << static_cast<unsigned char>(exponent + 128);
    }
  }
}

}
}
