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

#include "image.h"
#include "srgb.h"

namespace amber {
namespace post_process {

template <typename HDR, typename LDR = SRGB>
class Gamma {
private:
  using hdr_image_type = Image<HDR>;
  using hdr_value_type = typename HDR::value_type;
  using ldr_image_type = Image<LDR>;
  using ldr_value_type = typename LDR::value_type;

private:
  hdr_value_type gamma_;

public:
  Gamma() noexcept : gamma_(2.2) {}
  explicit Gamma(hdr_value_type gamma) noexcept : gamma_(gamma) {}

  ldr_image_type operator()(const hdr_image_type& hdr) const {
    const auto& width = hdr.width();
    const auto& height = hdr.height();

    ldr_image_type ldr(width, height);
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto& r = hdr.at(i, j).r();
        const auto& g = hdr.at(i, j).g();
        const auto& b = hdr.at(i, j).b();
        ldr.at(i, j) = LDR(
          std::min<hdr_value_type>(255, 255 * std::pow(r, 1 / gamma_)),
          std::min<hdr_value_type>(255, 255 * std::pow(g, 1 / gamma_)),
          std::min<hdr_value_type>(255, 255 * std::pow(b, 1 / gamma_))
        );
      }
    }

    return ldr;
  }
};

}
}
