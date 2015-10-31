/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/sampler.h"
#include "geometry/aabb.h"
#include "geometry/first_ray.h"
#include "geometry/hit.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Primitive {
public:
  using aabb_type      = AABB<RealType>;
  using first_ray_type = FirstRay<RealType>;
  using hit_type       = Hit<RealType>;
  using ray_type       = Ray<RealType>;
  using real_type      = RealType;

  virtual real_type surfaceArea() const noexcept = 0;
  virtual aabb_type aabb() const noexcept = 0;
  virtual hit_type intersect(ray_type const&) const noexcept = 0;
  virtual first_ray_type sampleFirstRay(Sampler*) const = 0;
};

}
}
}
