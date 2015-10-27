/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>

#include "base/constant.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Disk : public Primitive<RealType> {
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
  using hit_type       = typename Primitive<RealType>::hit_type;
  using first_ray_type = typename Primitive<RealType>::first_ray_type;
  using ray_type       = typename Primitive<RealType>::ray_type;

  using vector3_type   = Vector3<RealType>;

private:
  vector3_type center_, normal_;
  RealType radius_;

public:
  Disk(vector3_type const& center,
       vector3_type const& normal,
       RealType radius) noexcept
    : center_(center), normal_(normalize(normal)), radius_(radius) {}

  RealType surfaceArea() const noexcept {
    return static_cast<RealType>(kPI) * radius_ * radius_;
  }

  aabb_type aabb() const noexcept {
    auto const factor =
      vector3_type(std::sqrt(1 - normal_.x() * normal_.x()),
                   std::sqrt(1 - normal_.y() * normal_.y()),
                   std::sqrt(1 - normal_.z() * normal_.z()));

    return aabb_type(center_ - radius_ * factor, center_ + radius_ * factor);
  }

  hit_type intersect(ray_type const& ray) const noexcept {
     auto const cos_theta = dot(ray.direction, normal_);
     if (cos_theta == 0) {
       return hit_type();
     }

     auto const t = dot(center_ - ray.origin, normal_) / cos_theta;
     if (t < kEPS) {
       return hit_type();
     }

     auto const squared_distance =
       (ray.origin + t * ray.direction - center_).squaredLength();
     if (squared_distance > radius_ * radius_) {
       return hit_type();
     }

     return hit_type(ray.origin + t * ray.direction, normal_, t);
  }

  first_ray_type sampleFirstRay(Sampler* sampler) const {
    auto const radius = std::sqrt(sampler->uniform(radius_ * radius_));

    vector3_type u, v;
    std::tie(u, v) = orthonormalBasis(normal_);

    RealType x, y;
    std::tie(x, y) = sampler->circle<RealType>();

    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = sampler->hemispherePSA(u, v, normal_);

    return first_ray_type(center_ + (u * x + v * y) * radius,
                          direction_o,
                          normal_);
  }
};

}
}
}
