/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "aabb.h"
#include "hit.h"
#include "ray.h"
#include "sampler.h"

namespace amber {

template <typename RealType>
class Primitive
{
public:
  using aabb_type = AABB<RealType>;
  using hit_type  = Hit<RealType>;
  using ray_type  = Ray<RealType>;
  using real_type = RealType;

  virtual RealType SurfaceArea() const noexcept = 0;
  virtual aabb_type BoundingBox() const noexcept = 0;
  virtual hit_type Intersect(ray_type const&) const noexcept = 0;
  virtual ray_type SamplePoint(Sampler*) const = 0;
};

}
