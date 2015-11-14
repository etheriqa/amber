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

#include <algorithm>
#include <cmath>
#include <functional>

#include "core/constant.h"
#include "core/image.h"
#include "core/rgb.h"

namespace amber {
namespace post_process {

template <typename HDR>
class Normalizer {
private:
  using hdr_image_type = core::Image<HDR>;
  using hdr_value_type = typename HDR::value_type;

  using evaluator_type = std::function<hdr_value_type(HDR)>;

  evaluator_type evaluator_;

public:
  Normalizer() noexcept
    : evaluator_([](const auto& hdr){ return hdr.Max(); }) {}

  explicit Normalizer(evaluator_type evaluator) noexcept
    : evaluator_(evaluator) {}

  hdr_image_type operator()(const hdr_image_type& image) const {
    auto clone = image;
    return (*this)(clone);
  }

  hdr_image_type& operator()(hdr_image_type& image) const
  {
    const auto& width = image.width();
    const auto& height = image.height();

    hdr_value_type max = 0;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto value = evaluator_(image.at(i, j));
        if (std::isfinite(value)) {
          max = std::max(max, value);
        }
      }
    }

    if (max == 0) {
      return image;
    }

    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        image.at(i, j) /= max;
      }
    }

    return image;
  }
};

}
}
