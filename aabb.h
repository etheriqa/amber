#pragma once

#include <cmath>
#include <limits>
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

  AABB(const vector3_type& min, const vector3_type& max) :
    min(min), max(max)
  {}

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

  vector3_type center() const noexcept
  {
    return (min + max) / static_cast<real_type>(2);
  }

  bool intersect(const ray_type& ray) const noexcept
  {
    real_type tmin, tmax;

    {
      const auto t0 = (min.x - ray.origin.x) / ray.direction.x;
      const auto t1 = (max.x - ray.origin.x) / ray.direction.x;
      tmin = std::min(t0, t1);
      tmax = std::max(t0, t1);
    }

    {
      const auto t0 = (min.y - ray.origin.y) / ray.direction.y;
      const auto t1 = (max.y - ray.origin.y) / ray.direction.y;
      tmin = std::max(tmin, std::min(t0, t1));
      tmax = std::min(tmax, std::max(t0, t1));
      if (tmin > tmax || tmax < kEPS) {
        return false;
      }
    }

    {
      const auto t0 = (min.z - ray.origin.z) / ray.direction.z;
      const auto t1 = (max.z - ray.origin.z) / ray.direction.z;
      tmin = std::max(tmin, std::min(t0, t1));
      tmax = std::min(tmax, std::max(t0, t1));
      if (tmin > tmax || tmax < kEPS) {
        return false;
      }
    }

    return true;
  }
};

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
