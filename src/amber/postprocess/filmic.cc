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

#include "amber/postprocess/filmic.h"
#include "amber/prelude/image.h"
#include "amber/prelude/vector3.h"

namespace amber {
namespace postprocess {

namespace {

static constexpr hdr_value_type kA = 0.22; // shoulder strength
static constexpr hdr_value_type kB = 0.30; // linear strength
static constexpr hdr_value_type kC = 0.10; // linear angle
static constexpr hdr_value_type kD = 0.20; // toe strength
static constexpr hdr_value_type kE = 0.01; // toe numerator
static constexpr hdr_value_type kF = 0.30; // toe denominator
static constexpr hdr_value_type kW = 0.70; // linear white point
static constexpr hdr_value_type kExposure = 16.0;

}

HDRImage
Filmic::operator()(const HDRImage& input) const
{
  return (*this)(static_cast<HDRImage>(input));
}

HDRImage
Filmic::operator()(HDRImage&& input) const
{
  for (pixel_size_type j = 0; j < input.Height(); j++) {
    for (pixel_size_type i = 0; i < input.Width(); i++) {
      auto& p = input[Pixel(i, j)];
      p = Map(p * kExposure) / Map(HDR(kW));
    }
  }

  return input;
}

const HDR
Filmic::Map(const HDR& hdr) noexcept
{
  return
    (hdr * (hdr * kA + kB * kC) + kD * kE) /
    (hdr * (hdr * kA + kB) + kD * kF) -
    kE / kF;
}

}
}
