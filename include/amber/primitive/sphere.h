/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "algebra.h"
#include "constant.h"
#include "primitive.h"

namespace amber {
namespace primitive {

template <typename RealType>
class Sphere : public Primitive<RealType>
{
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
  using hit_type       = typename Primitive<RealType>::hit_type;
  using ray_type       = typename Primitive<RealType>::ray_type;

  using vector3_type   = Vector3<RealType>;

private:
  vector3_type center_;
  RealType radius_;

public:
  Sphere(vector3_type const& center, RealType radius) noexcept
    : center_(center), radius_(radius) {}

  RealType SurfaceArea() const noexcept {
    return 4 * static_cast<RealType>(kPI) * radius_ * radius_;
  }

  aabb_type BoundingBox() const noexcept {
    return aabb_type(center_ - radius_, center_ + radius_);
  }

  hit_type Intersect(ray_type const& ray) const noexcept {
    auto const a = static_cast<RealType>(1);
    auto const b = -2 * Dot(center_ - ray.origin, ray.direction);
    auto const c = (center_ - ray.origin).SquaredLength() - radius_ * radius_;

    auto const solutions = SolveQuadratic(a, b, c);
    if (!solutions) {
      return hit_type();
    }

    auto const& alpha = std::get<0>(*solutions);
    auto const& beta = std::get<1>(*solutions);

    if (alpha > kEPS) {
      return hit_type(ray.origin + alpha * ray.direction,
                      ray.origin + alpha * ray.direction - center_,
                      alpha);
    }

    if (beta > kEPS) {
      return hit_type(ray.origin + beta * ray.direction,
                      ray.origin + beta * ray.direction - center_,
                      beta);
    }

    return hit_type();
  }

  ray_type SamplePoint(Sampler* sampler) const
  {
    auto const normal = sampler->sphereSA<RealType>();
    auto const origin = center_ + radius_ * normal;

    return ray_type(origin, normal);
  }
};

}
}
