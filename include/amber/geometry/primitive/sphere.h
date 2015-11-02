/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/algebra.h"
#include "base/constant.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Sphere : public Primitive<RealType> {
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
  using first_ray_type = typename Primitive<RealType>::first_ray_type;
  using hit_type       = typename Primitive<RealType>::hit_type;
  using ray_type       = typename Primitive<RealType>::ray_type;

  using vector3_type   = Vector3<RealType>;

private:
  vector3_type center_;
  RealType radius_;

public:
  Sphere(vector3_type const& center, RealType radius) noexcept
    : center_(center), radius_(radius) {}

  RealType surfaceArea() const noexcept {
    return 4 * static_cast<RealType>(kPI) * radius_ * radius_;
  }

  aabb_type aabb() const noexcept {
    return aabb_type(center_ - radius_, center_ + radius_);
  }

  hit_type intersect(ray_type const& ray) const noexcept {
    auto const a = static_cast<RealType>(1);
    auto const b = -2 * dot(center_ - ray.origin, ray.direction);
    auto const c = (center_ - ray.origin).squaredLength() - radius_ * radius_;

    bool hit;
    RealType alpha, beta;
    std::tie(hit, alpha, beta) = solve_quadratic(a, b, c);

    if (!hit) {
      return hit_type();
    }

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

  first_ray_type sampleFirstRay(Sampler* sampler) const {
    auto const normal = sampler->sphereSA<RealType>();

    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = sampler->hemispherePSA(normal);

    return first_ray_type(center_ + radius_ * normal,
                          direction_o,
                          normal);
  }
};

}
}
}
