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

#include "amber/postprocess/gamma.h"
#include "amber/prelude/image.h"
#include "amber/prelude/vector3.h"

namespace amber {
namespace postprocess {

Gamma::Gamma() noexcept
: gamma_(2.2)
{}

Gamma::Gamma(hdr_value_type gamma) noexcept
: gamma_(gamma)
{}

LDRImage
Gamma::operator()(const HDRImage& input) const
{
  LDRImage output(input.Width(), input.Height());
  for (pixel_size_type j = 0; j < input.Height(); j++) {
    for (pixel_size_type i = 0; i < input.Width(); i++) {
      const auto& hdr = input[Pixel(i, j)];
      auto& ldr = output[Pixel(i, j)];
      ldr = LDR(
        255 * std::min<hdr_value_type>(1, std::pow(hdr.X(), 1 / gamma_)),
        255 * std::min<hdr_value_type>(1, std::pow(hdr.Y(), 1 / gamma_)),
        255 * std::min<hdr_value_type>(1, std::pow(hdr.Z(), 1 / gamma_))
      );
    }
  }
  return output;
}

}
}
