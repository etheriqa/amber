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

#pragma once

#include <tuple>
#include <vector>

#include "amber/rendering/forward.h"
#include "amber/scene/forward.h"

namespace amber {
namespace scene {

template <typename Radiant>
class Lens
{
public:
  virtual ~Lens() {}

  virtual std::vector<const Object<Radiant>*>
  ApertureObjects() const noexcept = 0;

  virtual std::tuple<const Object<Radiant>*, rendering::Leading<Radiant>, Pixel>
  GenerateRay(const rendering::Sensor& sensor, Sampler& sampler) const = 0;

  virtual PixelValue<Radiant>
  Response(const rendering::Sensor& sensor, const Ray& ray) const noexcept = 0;

  virtual const real_type
  PDFArea(const Vector3& point) const noexcept = 0;

  virtual const real_type
  PDFDirection(
    const rendering::Sensor& sensor,
    const Ray& ray
  ) const noexcept = 0;
};

}
}
