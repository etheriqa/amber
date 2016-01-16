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

#include <algorithm>
#include <limits>
#include <tuple>

#include <boost/operators.hpp>

#include "amber/prelude/ray.h"
#include "amber/prelude/vector3.h"

namespace amber {
namespace prelude {

template <typename T>
class AABB
: private boost::addable<AABB<T>>
, private boost::multipliable<AABB<T>>
{
public:
  static const AABB<T> Empty() noexcept;
  static const AABB<T> Universal() noexcept;

  AABB(const Vector3<T>& point) noexcept;
  AABB(const Vector3<T>& min, const Vector3<T>& max) noexcept;

  const Vector3<T>& Min() const noexcept { return min_; }
  const Vector3<T>& Max() const noexcept { return max_; }

  AABB<T>& operator+=(const AABB<T>& bb) noexcept;
  AABB<T>& operator*=(const AABB<T>& bb) noexcept;

private:
  Vector3<T> min_, max_;
};

template <typename T>
const Vector3<T>
Size(const AABB<T>& bb) noexcept;

template <typename T>
const T
SurfaceArea(const AABB<T>& bb) noexcept;

template <typename T>
std::tuple<bool, T, T>
Intersect(const AABB<T>& bb, const Ray<T>& ray, T t_max) noexcept;

template <>
std::tuple<bool, std::float_t, std::float_t>
Intersect(
  const AABB<std::float_t>& bb,
  const Ray<std::float_t>& ray,
  std::float_t t_max
) noexcept;

template <>
std::tuple<bool, std::double_t, std::double_t>
Intersect(
  const AABB<std::double_t>& bb,
  const Ray<std::double_t>& ray,
  std::double_t t_max
) noexcept;



template <typename T>
const AABB<T>
AABB<T>::Empty() noexcept
{
  return AABB<T>(
    Vector3<T>(std::numeric_limits<T>::max()),
    Vector3<T>(std::numeric_limits<T>::lowest())
  );
}

template <typename T>
const AABB<T>
AABB<T>::Universal() noexcept
{
  return AABB<T>(
    Vector3<T>(std::numeric_limits<T>::lowest()),
    Vector3<T>(std::numeric_limits<T>::max())
  );
}

template <typename T>
AABB<T>::AABB(const Vector3<T>& point) noexcept
: AABB(point, point)
{}

template <typename T>
AABB<T>::AABB(const Vector3<T>& min, const Vector3<T>& max) noexcept
: min_(min)
, max_(max)
{}

template <typename T>
AABB<T>&
AABB<T>::operator+=(const AABB<T>& bb) noexcept
{
  return this->operator=(AABB<T>(
    Vector3<T>(
      std::min(min_.X(), bb.min_.X()),
      std::min(min_.Y(), bb.min_.Y()),
      std::min(min_.Z(), bb.min_.Z())
    ),
    Vector3<T>(
      std::max(max_.X(), bb.max_.X()),
      std::max(max_.Y(), bb.max_.Y()),
      std::max(max_.Z(), bb.max_.Z())
    )
  ));
}

template <typename T>
AABB<T>&
AABB<T>::operator*=(const AABB<T>& bb) noexcept
{
  return this->operator=(AABB<T>(
    Vector3<T>(
      std::max(min_.X(), bb.min_.X()),
      std::max(min_.Y(), bb.min_.Y()),
      std::max(min_.Z(), bb.min_.Z())
    ),
    Vector3<T>(
      std::min(max_.X(), bb.max_.X()),
      std::min(max_.Y(), bb.max_.Y()),
      std::min(max_.Z(), bb.max_.Z())
    )
  ));
}

template <typename T>
const Vector3<T>
Size(const AABB<T>& bb) noexcept
{
  return Vector3<T>(
    bb.Max().X() - bb.Min().X(),
    bb.Max().Y() - bb.Min().Y(),
    bb.Max().Z() - bb.Min().Z()
  );
}

template <typename T>
const T
SurfaceArea(const AABB<T>& bb) noexcept
{
  const auto size = Size(bb);
  return 2 * (size.X() * size.Y() + size.Y() * size.Z() + size.Z() * size.X());
}

}
}
