#pragma once

#include <fstream>
#include "image.h"
#include "radiometry/srgb.h"

namespace amber {
namespace io {

void export_ppm(const char* filename, const Image<radiometry::SRGB>& image)
{
  std::ofstream ofs(filename, std::ofstream::trunc);

  ofs << "P3" << std::endl;
  ofs << image.m_width << " " << image.m_height << std::endl;
  ofs << 255 << std::endl;

  for (size_t j = 0; j < image.m_height; j++) {
    for (size_t i = 0; i < image.m_width; i++) {
      ofs << static_cast<size_t>(image.pixel(i, j).r()) << ' ' << static_cast<size_t>(image.pixel(i, j).g()) << ' ' << static_cast<size_t>(image.pixel(i, j).b()) << std::endl;
    }
  }
}

}
}
