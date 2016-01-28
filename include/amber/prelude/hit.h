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
class Hit
{
public:
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

  const Vector3<T>& Position() const noexcept { return position_; }
  const UnitVector3<T>& Normal() const noexcept { return normal_; }
  const T Distance() const noexcept { return distance_; }

  operator bool() const noexcept;

private:
  Vector3<T> position_;
  UnitVector3<T> normal_;
  T distance_;
};



template <typename T>
Hit<T>::Hit() noexcept
: position_()
, normal_()
, distance_(std::numeric_limits<T>::quiet_NaN())
{}

template <typename T>
Hit<T>::Hit(
  const Vector3<T>& position,
  const Vector3<T>& normal,
  T distance
) noexcept
: position_(position)
, normal_(Normalize(normal))
, distance_(distance)
{}

template <typename T>
Hit<T>::Hit(
  const Vector3<T>& position,
  const UnitVector3<T>& normal,
  T distance
) noexcept
: position_(position)
, normal_(normal)
, distance_(distance)
{}

template <typename T>
Hit<T>::operator bool() const noexcept
{
  return std::isfinite(distance_);
}

}
}
