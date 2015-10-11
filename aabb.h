#pragma once

#include <cmath>
#include <limits>
#include <ostream>
#include <tuple>
#include "constant.h"
#include "ray.h"
#include "vector.h"

namespace amber {

template <typename RealType>
struct AABB
{
  using real_type    = RealType;

  using aabb_type    = AABB<RealType>;
  using ray_type     = Ray<real_type>;
  using vector3_type = Vector3<real_type>;

  vector3_type min, max;

  static constexpr aabb_type empty() noexcept
  {
    return aabb_type(
      vector3_type(std::numeric_limits<real_type>::max()),
      vector3_type(std::numeric_limits<real_type>::lowest())
    );
  }

  static constexpr aabb_type universal() noexcept
  {
    return aabb_type(
      vector3_type(std::numeric_limits<real_type>::lowest()),
      vector3_type(std::numeric_limits<real_type>::max())
    );
  }

  explicit AABB() : AABB(empty()) {}

  explicit AABB(const vector3_type& point) : min(point), max(point) {}

  AABB(const vector3_type& min, const vector3_type& max) :
    min(min), max(max)
  {}

  operator bool() const noexcept
  {
    return min.x <= max.x && min.y <= max.y && min.z <= max.z;
  }

  aabb_type& operator+=(const aabb_type& a) noexcept
  {
    *this = *this + a;
    return *this;
  }

  aabb_type& operator*=(const aabb_type& a) noexcept
  {
    *this = *this * a;
    return *this;
  }

  real_type surface_area() const noexcept
  {
    const auto x = max.x - min.x;
    const auto y = max.y - min.y;
    const auto z = max.z - min.z;
    return 2 * (x * y + y * z + z * x);
  }

  std::tuple<bool, real_type, real_type> intersect(const ray_type& ray, real_type t_max) const noexcept
  {
    real_type t_min = static_cast<real_type>(kEPS);

    {
      const auto t0 = (min.x - ray.origin.x) / ray.direction.x;
      const auto t1 = (max.x - ray.origin.x) / ray.direction.x;
      t_min = std::max(t_min, std::min(t0, t1));
      t_max = std::min(t_max, std::max(t0, t1));
      if (t_min > t_max) {
        return std::make_tuple(false, 0, 0);
      }
    }

    {
      const auto t0 = (min.y - ray.origin.y) / ray.direction.y;
      const auto t1 = (max.y - ray.origin.y) / ray.direction.y;
      t_min = std::max(t_min, std::min(t0, t1));
      t_max = std::min(t_max, std::max(t0, t1));
      if (t_min > t_max) {
        return std::make_tuple(false, 0, 0);
      }
    }

    {
      const auto t0 = (min.z - ray.origin.z) / ray.direction.z;
      const auto t1 = (max.z - ray.origin.z) / ray.direction.z;
      t_min = std::max(t_min, std::min(t0, t1));
      t_max = std::min(t_max, std::max(t0, t1));
      if (t_min > t_max) {
        return std::make_tuple(false, 0, 0);
      }
    }

    return std::make_tuple(true, t_min, t_max);
  }
};

template <typename RealType>
std::ostream& operator<<(std::ostream& os, const AABB<RealType>& aabb)
{
  os << "([" << aabb.min.x << ", " << aabb.max.x << "], [" << aabb.min.y << ", " << aabb.max.y << "], [" << aabb.min.z << ", " << aabb.max.z << "])";
  return os;
}

template <typename RealType>
AABB<RealType> operator+(const AABB<RealType>& a, const AABB<RealType>& b) noexcept
{
  return AABB<RealType>(
    Vector3<RealType>(
      std::min(a.min.x, b.min.x),
      std::min(a.min.y, b.min.y),
      std::min(a.min.z, b.min.z)
    ),
    Vector3<RealType>(
      std::max(a.max.x, b.max.x),
      std::max(a.max.y, b.max.y),
      std::max(a.max.z, b.max.z)
    )
  );
}

template <typename RealType>
AABB<RealType> operator*(const AABB<RealType>& a, const AABB<RealType>& b) noexcept
{
  return AABB<RealType>(
    Vector3<RealType>(
      std::max(a.min.x, b.min.x),
      std::max(a.min.y, b.min.y),
      std::max(a.min.z, b.min.z)
    ),
    Vector3<RealType>(
      std::min(a.max.x, b.max.x),
      std::min(a.max.y, b.max.y),
      std::min(a.max.z, b.max.z)
    )
  );
}

}
