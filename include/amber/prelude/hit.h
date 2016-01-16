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

#include <cmath>
#include <limits>

#include "amber/prelude/vector3.h"

namespace amber {
namespace prelude {

template <typename T>
struct Hit
{
  Vector3<T> position;
  UnitVector3<T> normal;
  T distance;

  Hit() noexcept;
  Hit(
    const Vector3<T>& position,
    const Vector3<T>& normal,
    T distance
  ) noexcept;
  Hit(
    const Vector3<T>& position,
    const UnitVector3<T>& normal,
    T distance
  ) noexcept;

  operator bool() const noexcept;
};



template <typename T>
Hit<T>::Hit() noexcept
: position()
, normal()
, distance(std::numeric_limits<T>::quiet_NaN())
{}

template <typename T>
Hit<T>::Hit(
  const Vector3<T>& position,
  const Vector3<T>& normal,
  T distance
) noexcept
: Hit(position, Normalize(normal), distance)
{}

template <typename T>
Hit<T>::Hit(
  const Vector3<T>& position,
  const UnitVector3<T>& normal,
  T distance
) noexcept
: position(position)
, normal(normal)
, distance(distance)
{}

template <typename T>
Hit<T>::operator bool() const noexcept
{
  return std::isfinite(distance);
}

}
}
