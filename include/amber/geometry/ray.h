/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "writer.h"
#include "geometry/vector.h"

namespace amber {
namespace geometry {

template <typename RealType>
struct Ray : public Writer {
  using vector3_type = Vector3<RealType>;

  vector3_type origin, direction;

  Ray(vector3_type const& origin, vector3_type const& direction) noexcept
    : origin(origin), direction(normalize(direction)) {}

  void write(std::ostream& os) const noexcept {
    os << "Ray(origin=" << origin << ", direction=" << direction << ")";
  }
};

}
}
