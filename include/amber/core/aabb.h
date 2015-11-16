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

#include <boost/operators.hpp>
#include <immintrin.h>

#include "core/constant.h"
#include "core/ray.h"
#include "core/writer.h"

namespace amber {
namespace core {

template <typename RealType>
class AABB
: public Writer,
  private boost::addable<AABB<RealType>>,
  private boost::multipliable<AABB<RealType>>
{
public:
  using ray_type     = Ray<RealType>;
  using vector3_type = Vector3<RealType>;

private:
  vector3_type min_, max_;

public:
  AABB<RealType> const static constexpr empty() noexcept
  {
    return AABB<RealType>(
      vector3_type(std::numeric_limits<RealType>::max()),
      vector3_type(std::numeric_limits<RealType>::lowest())
    );
  }

  AABB<RealType> const static constexpr universal() noexcept
  {
    return AABB<RealType>(
      vector3_type(std::numeric_limits<RealType>::lowest()),
      vector3_type(std::numeric_limits<RealType>::max())
    );
  }

  AABB() noexcept : AABB(empty()) {}

  explicit AABB(vector3_type const& point) noexcept
  : min_(point), max_(point) {}

  AABB(vector3_type const& min, vector3_type const& max) noexcept
  : min_(min), max_(max) {}

  vector3_type const& min() const noexcept { return min_; }
  vector3_type const& max() const noexcept { return max_; }

  vector3_type& min() noexcept { return min_; }
  vector3_type& max() noexcept { return max_; }

  operator bool() const noexcept
  {
    return min_.x() <= max_.x() && min_.y() <= max_.y() && min_.z() <= max_.z();
  }

  AABB<RealType>& operator+=(AABB<RealType> const& bb) noexcept
  {
    return this->operator=(AABB<RealType>(
      Vector3<RealType>(
        std::min(min_.x(), bb.min_.x()),
        std::min(min_.y(), bb.min_.y()),
        std::min(min_.z(), bb.min_.z())
      ),
      Vector3<RealType>(
        std::max(max_.x(), bb.max_.x()),
        std::max(max_.y(), bb.max_.y()),
        std::max(max_.z(), bb.max_.z())
      )
    ));
  }

  AABB<RealType>& operator*=(AABB<RealType> const& bb) noexcept
  {
    return this->operator=(AABB<RealType>(
      Vector3<RealType>(
        std::max(min_.x(), bb.min_.x()),
        std::max(min_.y(), bb.min_.y()),
        std::max(min_.z(), bb.min_.z())
      ),
      Vector3<RealType>(
        std::min(max_.x(), bb.max_.x()),
        std::min(max_.y(), bb.max_.y()),
        std::min(max_.z(), bb.max_.z())
      )
    ));
  }

  void Write(std::ostream& os) const noexcept
  {
    os
      << "([" << min_.x()
      << ", " << max_.x()
      << "], [" << min_.y()
      << ", " << max_.y()
      << "], [" << min_.z()
      << ", " << max_.z()
      << "])";
  }

  RealType SurfaceArea() const noexcept
  {
    auto const x = max_.x() - min_.x();
    auto const y = max_.y() - min_.y();
    auto const z = max_.z() - min_.z();
    return 2 * (x * y + y * z + z * x);
  }

  std::tuple<bool, RealType, RealType>
  Intersect(ray_type const& ray, RealType t_max) const noexcept
  {
    RealType t_min = static_cast<RealType>(kEPS);

    {
      auto const t0 = (min_.x() - ray.origin.x()) / ray.direction.x();
      auto const t1 = (max_.x() - ray.origin.x()) / ray.direction.x();
      t_min = std::max(t_min, std::min(t0, t1));
      t_max = std::min(t_max, std::max(t0, t1));
      if (t_min > t_max) {
        return std::make_tuple(false, 0, 0);
      }
    }

    {
      auto const t0 = (min_.y() - ray.origin.y()) / ray.direction.y();
      auto const t1 = (max_.y() - ray.origin.y()) / ray.direction.y();
      t_min = std::max(t_min, std::min(t0, t1));
      t_max = std::min(t_max, std::max(t0, t1));
      if (t_min > t_max) {
        return std::make_tuple(false, 0, 0);
      }
    }

    {
      auto const t0 = (min_.z() - ray.origin.z()) / ray.direction.z();
      auto const t1 = (max_.z() - ray.origin.z()) / ray.direction.z();
      t_min = std::max(t_min, std::min(t0, t1));
      t_max = std::min(t_max, std::max(t0, t1));
      if (t_min > t_max) {
        return std::make_tuple(false, 0, 0);
      }
    }

    return std::make_tuple(true, t_min, t_max);
  }
};

template <>
std::tuple<bool, std::double_t, std::double_t>
AABB<std::double_t>::Intersect(
  Ray<std::double_t> const& ray,
  std::double_t t_max
) const noexcept
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
    _mm256_setr_pd(min_.x(), min_.y(), min_.z(), 0);
  auto const aabb_max =
    _mm256_setr_pd(max_.x(), max_.y(), max_.z(), 0);

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
}
