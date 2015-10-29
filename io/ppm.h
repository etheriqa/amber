/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <fstream>
#include <string>

#include "camera/image.h"
#include "radiometry/srgb.h"

namespace amber {
namespace io {

void export_ppm(std::string const& filename,
                camera::Image<radiometry::SRGB> const& image) {
  std::ofstream ofs(filename, std::ofstream::trunc);

  ofs << "P3" << std::endl;
  ofs << image.width() << " " << image.height() << std::endl;
  ofs << 255 << std::endl;

  for (size_t j = 0; j < image.height(); j++) {
    for (size_t i = 0; i < image.width(); i++) {
      ofs
        << static_cast<size_t>(image.at(i, j).r())
        << ' '
        << static_cast<size_t>(image.at(i, j).g())
        << ' '
        << static_cast<size_t>(image.at(i, j).b())
        << std::endl;
    }
  }
}

}
}
