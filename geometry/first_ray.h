/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "geometry/ray.h"

namespace amber {
namespace geometry {

template <typename RealType>
struct FirstRay : public Ray<RealType> {
  using vector3_type = typename Ray<RealType>::vector3_type;

  vector3_type normal;

  FirstRay(vector3_type const& origin,
           vector3_type const& direction,
           vector3_type const& normal) noexcept
    : Ray<RealType>(origin, direction), normal(normalize(normal)) {}

  void write(std::ostream& os) const noexcept {
    os
      << "FirstRay(origin=" << this->origin
      << ", direction=" << this->direction
      << ", normal=" << normal
      << ")";
  }
};

}
}
