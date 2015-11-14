// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <cmath>
#include <fstream>
#include <string>

#include "core/image.h"
#include "core/rgb.h"

namespace amber {
namespace io {

template <typename RealType>
void export_rgbe(std::string const& filename,
                 core::Image<core::RGB<RealType>> const& image) {
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
