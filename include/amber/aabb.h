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

#include <cmath>
#include <limits>
#include <ostream>
#include <tuple>

#include <immintrin.h>

#include "constant.h"
#include "ray.h"
#include "writer.h"

namespace amber {

template <typename RealType>
struct AABB : public Writer
{
  using ray_type     = Ray<RealType>;
  using vector3_type = Vector3<RealType>;

  vector3_type min, max;

  AABB<RealType> static constexpr empty() noexcept
  {
    return AABB<RealType>(
      vector3_type(std::numeric_limits<RealType>::max()),
      vector3_type(std::numeric_limits<RealType>::lowest())
    );
  }

  AABB<RealType> static constexpr universal() noexcept
  {
    return AABB<RealType>(
      vector3_type(std::numeric_limits<RealType>::lowest()),
      vector3_type(std::numeric_limits<RealType>::max())
    );
  }

  AABB() noexcept : AABB(empty()) {}

  explicit AABB(vector3_type const& point) noexcept
  : min(point), max(point) {}

  AABB(vector3_type const& min, vector3_type const& max) noexcept
  : min(min), max(max) {}

  void Write(std::ostream& os) const noexcept {
    os
      << "([" << min.x()
      << ", " << max.x()
      << "], [" << min.y()
      << ", " << max.y()
      << "], [" << min.z()
      << ", " << max.z()
      << "])";
  }

  operator bool() const noexcept
  {
    return min.x() <= max.x() && min.y() <= max.y() && min.z() <= max.z();
  }

  AABB<RealType>& operator+=(AABB<RealType> const& a) noexcept
  {
    return *this = *this + a;
  }

  AABB<RealType>& operator*=(AABB<RealType> const& a) noexcept
  {
    return *this = *this * a;
  }

  RealType SurfaceArea() const noexcept
  {
    auto const x = max.x() - min.x();
    auto const y = max.y() - min.y();
    auto const z = max.z() - min.z();
    return 2 * (x * y + y * z + z * x);
  }

  std::tuple<bool, RealType, RealType>
  Intersect(ray_type const& ray, RealType t_max) const noexcept
  {
    return RayAABBIntersection(ray, *this, t_max);
  }
};

template <typename RealType>
AABB<RealType>
operator+(AABB<RealType> const& a, AABB<RealType> const& b) noexcept
{
  return
    AABB<RealType>(Vector3<RealType>(std::min(a.min.x(), b.min.x()),
                                     std::min(a.min.y(), b.min.y()),
                                     std::min(a.min.z(), b.min.z())),
                   Vector3<RealType>(std::max(a.max.x(), b.max.x()),
                                     std::max(a.max.y(), b.max.y()),
                                     std::max(a.max.z(), b.max.z())));
}

template <typename RealType>
AABB<RealType>
operator*(AABB<RealType> const& a, AABB<RealType> const& b) noexcept
{
  return
    AABB<RealType>(Vector3<RealType>(std::max(a.min.x(), b.min.x()),
                                     std::max(a.min.y(), b.min.y()),
                                     std::max(a.min.z(), b.min.z())),
                   Vector3<RealType>(std::min(a.max.x(), b.max.x()),
                                     std::min(a.max.y(), b.max.y()),
                                     std::min(a.max.z(), b.max.z())));
}

template <typename RealType>
std::tuple<bool, RealType, RealType>
RayAABBIntersection(
  Ray<RealType> const& ray,
  AABB<RealType> const& aabb,
  RealType t_max
) noexcept
{
  auto t_min = static_cast<RealType>(kEPS);

  {
    auto const t0 = (aabb.min.x() - ray.origin.x()) / ray.direction.x();
    auto const t1 = (aabb.max.x() - ray.origin.x()) / ray.direction.x();
    t_min = std::max(t_min, std::min(t0, t1));
    t_max = std::min(t_max, std::max(t0, t1));
    if (t_min > t_max) {
      return std::make_tuple(false, 0, 0);
    }
  }

  {
    auto const t0 = (aabb.min.y() - ray.origin.y()) / ray.direction.y();
    auto const t1 = (aabb.max.y() - ray.origin.y()) / ray.direction.y();
    t_min = std::max(t_min, std::min(t0, t1));
    t_max = std::min(t_max, std::max(t0, t1));
    if (t_min > t_max) {
      return std::make_tuple(false, 0, 0);
    }
  }

  {
    auto const t0 = (aabb.min.z() - ray.origin.z()) / ray.direction.z();
    auto const t1 = (aabb.max.z() - ray.origin.z()) / ray.direction.z();
    t_min = std::max(t_min, std::min(t0, t1));
    t_max = std::min(t_max, std::max(t0, t1));
    if (t_min > t_max) {
      return std::make_tuple(false, 0, 0);
    }
  }

  return std::make_tuple(true, t_min, t_max);
}

template <>
std::tuple<bool, double, double>
RayAABBIntersection(
  Ray<double> const& ray,
  AABB<double> const& aabb,
  double t_max
) noexcept
{
  double t_min = kEPS;

  auto const ray_origin =
    _mm256_setr_pd(ray.origin.x(), ray.origin.y(), ray.origin.z(), 0);
  auto const inverse_ray_direction =
    _mm256_div_pd(_mm256_setr_pd(1, 1, 1, 1),
                  _mm256_setr_pd(ray.direction.x(),
                                 ray.direction.y(),
                                 ray.direction.z(),
                                 1));
  auto const aabb_min =
    _mm256_setr_pd(aabb.min.x(),aabb.min.y(), aabb.min.z(), 0);
  auto const aabb_max =
    _mm256_setr_pd(aabb.max.x(), aabb.max.y(), aabb.max.z(), 0);

  auto const t0 =
    _mm256_mul_pd(_mm256_sub_pd(aabb_min, ray_origin), inverse_ray_direction);
  auto const t1 =
    _mm256_mul_pd(_mm256_sub_pd(aabb_max, ray_origin), inverse_ray_direction);

  double t_mins[4], t_maxs[4];
  _mm256_store_pd(t_mins, _mm256_min_pd(t0, t1));
  _mm256_store_pd(t_maxs, _mm256_max_pd(t0, t1));

  t_min = std::max({t_min, t_mins[0], t_mins[1], t_mins[2]});
  t_max = std::min({t_max, t_maxs[0], t_maxs[1], t_maxs[2]});

  return std::make_tuple(t_min <= t_max, t_min, t_max);
}

}
