/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>
#include <fstream>
#include <string>

#include "image.h"
#include "rgb.h"

namespace amber {
namespace io {

template <typename RealType>
void export_rgbe(std::string const& filename,
                 Image<RGB<RealType>> const& image) {
  std::ofstream ofs(filename, std::ofstream::trunc);

  ofs << "#?RADIANCE" << std::endl;
  ofs << "FORMAT=32-bit_rle_rgbe" << std::endl << std::endl;
  ofs << "-Y " << image.height() << " +X " << image.width() << std::endl;

  for (size_t j = 0; j < image.height(); j++) {
    for (size_t i = 0; i < image.width(); i++) {
      auto const& p = image.at(i, j);

      int exponent;
      auto const significand = std::frexp(p.Max(), &exponent) * 256 / p.Max();

      ofs << static_cast<unsigned char>(significand * p.r());
      ofs << static_cast<unsigned char>(significand * p.g());
      ofs << static_cast<unsigned char>(significand * p.b());
      ofs << static_cast<unsigned char>(exponent + 128);
    }
  }
}

}
}
