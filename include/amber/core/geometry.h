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

#include "vector3.h"

namespace amber {
namespace core {

template <typename RealType>
Vector3<RealType> const
PerfectReflection(
  Vector3<RealType> const& incident,
  Vector3<RealType> const& normal
) noexcept
{
  return PerfectReflection(incident, normal, Dot(incident, normal));
}

template <typename RealType>
Vector3<RealType> const
PerfectReflection(
  Vector3<RealType> const& incident,
  Vector3<RealType> const& normal,
  RealType signed_cos_theta
) noexcept
{
  return 2 * signed_cos_theta * normal - incident;
}

template <typename RealType>
RealType const
GeometryFactor(
  Vector3<RealType> const& px,
  Vector3<RealType> const& py,
  Vector3<RealType> const& nx,
  Vector3<RealType> const& ny
) noexcept
{
  auto const xy = py - px;
  auto const squared_distance = SquaredLength(xy);
  auto const direction = xy / std::sqrt(squared_distance);
  return GeometryFactor(direction, squared_distance, nx, ny);
}

template <typename RealType>
RealType const
GeometryFactor(
  Vector3<RealType> const& direction,
  RealType squared_distance,
  Vector3<RealType> const& nx,
  Vector3<RealType> const& ny
) noexcept
{
  return std::abs(Dot(direction, nx) * Dot(direction, ny) / squared_distance);
}


}
}
