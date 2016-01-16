// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include <cmath>
#include <limits>

#include "amber/postprocess/reinhard.h"
#include "amber/prelude/accumulator.h"
#include "amber/prelude/image.h"
#include "amber/prelude/vector3.h"

namespace amber {
namespace postprocess {

Reinhard::Reinhard() noexcept
: key_(0.18)
{}

Reinhard::Reinhard(hdr_value_type key) noexcept
: key_(key)
{}

HDRImage
Reinhard::operator()(const HDRImage& input) const
{
  return (*this)(static_cast<HDRImage>(input));
}

HDRImage
Reinhard::operator()(HDRImage&& input) const
{
  Accumulator<hdr_value_type> log_sum_luminance(0);
  for (pixel_size_type j = 0; j < input.Height(); j++) {
    for (pixel_size_type i = 0; i < input.Width(); i++) {
      log_sum_luminance += std::log(
        std::numeric_limits<hdr_value_type>::min() +
        Luminance(input[Pixel(i, j)])
      );
    }
  }
  const auto log_average_luminance = std::exp(log_sum_luminance.Mean());

  for (pixel_size_type j = 0; j < input.Height(); j++) {
    for (pixel_size_type i = 0; i < input.Width(); i++) {
      auto& hdr = input[Pixel(i, j)];
      hdr *= key_ / log_average_luminance;
      hdr /= static_cast<HDR>(1) + hdr;
    }
  }

  return input;
}

const hdr_value_type
Reinhard::Luminance(const HDR& hdr) noexcept
{
  return Dot(HDR(0.27, 0.67, 0.06), hdr);
}

}
}
