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

#include "amber/prelude/vector3.h"

namespace amber {
namespace prelude {

template <typename T>
const UnitVector3<T>
PerfectReflection(
  const UnitVector3<T>& incident,
  const UnitVector3<T>& normal
) noexcept
{
  return PerfectReflection(incident, normal, Dot(incident, normal));
}

template <typename T>
const UnitVector3<T>
PerfectReflection(
  const UnitVector3<T>& incident,
  const UnitVector3<T>& normal,
  T signed_cos_theta
) noexcept
{
  return UnitVector3<T>(2 * signed_cos_theta * normal - incident);
}

template <typename T>
const T
GeometryFactor(
  const Vector3<T>& px,
  const Vector3<T>& py,
  const UnitVector3<T>& nx,
  const UnitVector3<T>& ny
) noexcept
{
  const auto xy = py - px;
  const auto squared_distance = SquaredLength(xy);
  const auto direction =
    static_cast<UnitVector3<T>>(xy / std::sqrt(squared_distance));
  return GeometryFactor(direction, squared_distance, nx, ny);
}

template <typename T>
const T
GeometryFactor(
  const UnitVector3<T>& direction,
  T squared_distance,
  const UnitVector3<T>& nx,
  const UnitVector3<T>& ny
) noexcept
{
  return std::abs(Dot(direction, nx) * Dot(direction, ny) / squared_distance);
}

}
}
