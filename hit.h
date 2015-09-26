#pragma once

#include <cmath>
#include <limits>
#include <ostream>
#include "vector.h"

namespace amber {

template <typename RealType>
struct Hit
{
  using real_type    = RealType;

  using hit_type     = Hit<real_type>;
  using vector3_type = Vector3<real_type>;

  vector3_type position, normal;
  real_type distance;

  Hit() :
    position(vector3_type()),
    normal(vector3_type()),
    distance(std::numeric_limits<real_type>::quiet_NaN())
  {}

  Hit(const vector3_type& p, const vector3_type& n, real_type d) :
    position(p),
    normal(normalize(n)),
    distance(d)
  {}

  operator bool() const noexcept
  {
    return std::isfinite(distance);
  }
};

template <typename RealType>
std::ostream& operator<<(std::ostream& os, const Hit<RealType>& h)
{
  if (h) {
    os << "Hit(position=" << h.position << ", normal=" << h.normal << ", distance=" << h.distance << ")";
  } else {
    os << "Hit(none)";
  }
  return os;
}

}
