/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>
#include <limits>

#include "writer.h"
#include "vector.h"

namespace amber {

template <typename RealType>
struct Hit : public Writer
{
  using real_type    = RealType;

  using hit_type     = Hit<real_type>;
  using vector3_type = Vector3<real_type>;

  vector3_type position, normal;
  real_type distance;

  Hit() noexcept
  : position(vector3_type()),
    normal(vector3_type()),
    distance(std::numeric_limits<real_type>::quiet_NaN())
  {}

  Hit(const vector3_type& p, const vector3_type& n, real_type d) noexcept
  : position(p),
    normal(Normalize(n)),
    distance(d)
  {}

  operator bool() const noexcept
  {
    return std::isfinite(distance);
  }

  void Write(std::ostream& os) const noexcept
  {
    if (*this) {
      os
        << "Hit(position=" << position
        << ", normal=" << normal
        << ", distance=" << distance
        << ")";
    } else {
      os << "Hit(none)";
    }
  }
};

}
