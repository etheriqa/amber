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

#include <memory>

#include "amber/prelude/matrix3.h"
#include "amber/prelude/vector3.h"
#include "amber/scene/lens_basic.h"

namespace amber {
namespace scene {

template <typename Radiant>
std::unique_ptr<Lens<Radiant>>
MakeThinLens(
  const Matrix4& transform,
  real_type focal_length,
  real_type focus_distance,
  real_type radius,
  std::size_t n_blades
);

class BasicThin
{
public:
  BasicThin(
    const Matrix4& transform,
    real_type focal_length,
    real_type focus_distance,
    real_type radius,
    std::size_t n_blades
  ) noexcept;

  std::vector<const Primitive*>
  AperturePrimitives() const noexcept;

  std::tuple<const Primitive*, BasicEyeRay, Pixel>
  GenerateRay(const rendering::Sensor& sensor, Sampler& sampler) const;

  PixelValue<real_type>
  Response(const rendering::Sensor& sensor, const Ray& ray) const noexcept;

  const real_type
  PDFArea(const Vector3& point) const noexcept;

  const real_type
  PDFDirection(const rendering::Sensor& sensor, const Ray& ray) const noexcept;

private:
  Vector3 origin_;
  Matrix3 global_;
  Matrix3 local_;
  std::vector<std::unique_ptr<Primitive>> primitives_;
  real_type focus_distance_, sensor_distance_, p_area_;
};





template <typename Radiant>
std::unique_ptr<Lens<Radiant>>
MakeThinLens(
  const Matrix4& transform,
  real_type focal_length,
  real_type focus_distance,
  real_type radius,
  std::size_t n_blades
)
{
  return std::make_unique<BasicLensForwarder<Radiant, BasicThin>>(
    transform,
    focal_length,
    focus_distance,
    radius,
    n_blades
  );
}

}
}
