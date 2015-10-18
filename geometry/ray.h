#pragma once

#include <ostream>
#include "geometry/vector.h"

namespace amber {
namespace geometry {

template <typename RealType>
struct Ray
{
  using real_type    = RealType;

  using ray_type     = Ray<real_type>;
  using vector3_type = Vector3<real_type>;

  vector3_type origin, direction;

  Ray(const vector3_type& o, const vector3_type& d) :
    origin(o),
    direction(normalize(d))
  {}
};

template <typename RealType>
struct InitialRaySample
{
  using real_type    = RealType;

  using ray_type     = Ray<real_type>;
  using vector3_type = Vector3<real_type>;

  ray_type ray;
  vector3_type normal;

  InitialRaySample(const ray_type& ray, const vector3_type normal) :
    ray(ray), normal(normal)
  {}
};

template <typename RealType>
std::ostream& operator<<(std::ostream& os, const Ray<RealType>& r)
{
  os << "Ray(origin=" << r.origin << ", direction=" << r.direction << ")";
  return os;
}

}
}