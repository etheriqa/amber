/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>
#include <limits>
#include <ostream>
#include <tuple>

#include "immintrin.h"

#include "base/constant.h"
#include "geometry/ray.h"
#include "geometry/vector.h"

namespace amber {
namespace geometry {

template <typename RealType>
struct AABB {
  using aabb_type    = AABB<RealType>;
  using ray_type     = Ray<RealType>;
  using real_type    = RealType;
  using vector3_type = Vector3<RealType>;

  vector3_type min, max;

  static constexpr aabb_type empty() noexcept {
    return aabb_type(vector3_type(std::numeric_limits<real_type>::max()),
                     vector3_type(std::numeric_limits<real_type>::lowest()));
  }

  static constexpr aabb_type universal() noexcept {
    return aabb_type(vector3_type(std::numeric_limits<real_type>::lowest()),
                     vector3_type(std::numeric_limits<real_type>::max()));
  }

  AABB() noexcept : AABB(empty()) {}
  explicit AABB(const vector3_type& point) noexcept
    : min(point), max(point) {}
  AABB(const vector3_type& min, const vector3_type& max) noexcept
    : min(min), max(max) {}

  operator bool() const noexcept {
    return min.x() <= max.x() && min.y() <= max.y() && min.z() <= max.z();
  }

  aabb_type& operator+=(const aabb_type& a) noexcept {
    return *this = *this + a;
  }

  aabb_type& operator*=(const aabb_type& a) noexcept {
    return *this = *this * a;
  }

  real_type surface_area() const noexcept {
    const auto x = max.x() - min.x();
    const auto y = max.y() - min.y();
    const auto z = max.z() - min.z();
    return 2 * (x * y + y * z + z * x);
  }

  std::tuple<bool, real_type, real_type>
  intersect(const ray_type& ray, real_type t_max) const noexcept {
    return ray_aabb_intersection(ray, *this, t_max);
  }
};

template <typename RealType>
std::ostream&
operator<<(std::ostream& os, const AABB<RealType>& aabb) {
  os << "([" << aabb.min.x() << ", " << aabb.max.x() << "], [" << aabb.min.y() << ", " << aabb.max.y() << "], [" << aabb.min.z() << ", " << aabb.max.z() << "])";
  return os;
}

template <typename RealType>
AABB<RealType>
operator+(const AABB<RealType>& a, const AABB<RealType>& b) noexcept {
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
operator*(const AABB<RealType>& a, const AABB<RealType>& b) noexcept {
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
ray_aabb_intersection(const Ray<RealType>& ray,
                      const AABB<RealType>& aabb,
                      RealType t_max) noexcept {
  auto t_min = static_cast<RealType>(kEPS);

  {
    const auto t0 = (aabb.min.x() - ray.origin.x()) / ray.direction.x();
    const auto t1 = (aabb.max.x() - ray.origin.x()) / ray.direction.x();
    t_min = std::max(t_min, std::min(t0, t1));
    t_max = std::min(t_max, std::max(t0, t1));
    if (t_min > t_max) {
      return std::make_tuple(false, 0, 0);
    }
  }

  {
    const auto t0 = (aabb.min.y() - ray.origin.y()) / ray.direction.y();
    const auto t1 = (aabb.max.y() - ray.origin.y()) / ray.direction.y();
    t_min = std::max(t_min, std::min(t0, t1));
    t_max = std::min(t_max, std::max(t0, t1));
    if (t_min > t_max) {
      return std::make_tuple(false, 0, 0);
    }
  }

  {
    const auto t0 = (aabb.min.z() - ray.origin.z()) / ray.direction.z();
    const auto t1 = (aabb.max.z() - ray.origin.z()) / ray.direction.z();
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
ray_aabb_intersection(const Ray<double>& ray,
                      const AABB<double>& aabb,
                      double t_max) noexcept {
  auto t_min = static_cast<double>(kEPS);

  const auto ray_origin =
    _mm256_setr_pd(ray.origin.x(), ray.origin.y(), ray.origin.z(), 0);
  const auto inverse_ray_direction =
    _mm256_div_pd(_mm256_setr_pd(1, 1, 1, 1),
                  _mm256_setr_pd(ray.direction.x(),
                                 ray.direction.y(),
                                 ray.direction.z(),
                                 1));
  const auto aabb_min =
    _mm256_setr_pd(aabb.min.x(),aabb.min.y(), aabb.min.z(), 0);
  const auto aabb_max =
    _mm256_setr_pd(aabb.max.x(), aabb.max.y(), aabb.max.z(), 0);

  const auto t0 =
    _mm256_mul_pd(_mm256_sub_pd(aabb_min, ray_origin), inverse_ray_direction);
  const auto t1 =
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
