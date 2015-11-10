/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>

#include "constant.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Triangle : public Primitive<RealType> {
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
  using first_ray_type = typename Primitive<RealType>::first_ray_type;
  using hit_type       = typename Primitive<RealType>::hit_type;
  using ray_type       = typename Primitive<RealType>::ray_type;

  using vector3_type   = Vector3<RealType>;

private:
  vector3_type v0_, v1_, v2_;
  vector3_type normal_;

public:
  Triangle(vector3_type const& v0,
           vector3_type const& v1,
           vector3_type const& v2) noexcept
    : v0_(v0), v1_(v1), v2_(v2), normal_(normalize(cross(v1 - v0, v2 - v0))) {}

  RealType surfaceArea() const noexcept {
    return (cross(v1_ - v0_, v2_ - v0_)).length() / 2;
  }

  aabb_type aabb() const noexcept {
    return aabb_type(vector3_type(std::min({v0_.x(), v1_.x(), v2_.x()}),
                                  std::min({v0_.y(), v1_.y(), v2_.y()}),
                                  std::min({v0_.z(), v1_.z(), v2_.z()})),
                     vector3_type(std::max({v0_.x(), v1_.x(), v2_.x()}),
                                  std::max({v0_.y(), v1_.y(), v2_.y()}),
                                  std::max({v0_.z(), v1_.z(), v2_.z()})));
  }

  hit_type intersect(ray_type const& ray) const noexcept {
    auto const E1 = v1_ - v0_;
    auto const E2 = v2_ - v0_;

    auto const P = cross(ray.direction, E2);
    auto const det = dot(P, E1);

    auto const T = ray.origin - v0_;
    auto const u = dot(P, T) / det;
    if (u > 1 || u < 0) {
      return hit_type();
    }

    auto const Q = cross(T, E1);
    auto const v = dot(Q, ray.direction) / det;
    if (v > 1 || v < 0) {
      return hit_type();
    }

    if (u + v > 1) {
      return hit_type();
    }

    auto const t = dot(Q, E2) / det;
    if (t < kEPS) {
      return hit_type();
    }

    return hit_type(v0_ + u * E1 + v * E2,
                    normal_,
                    t);
  }

  first_ray_type sampleFirstRay(Sampler* sampler) const {
    auto u = sampler->uniform<RealType>();
    auto v = sampler->uniform<RealType>();

    if (u + v >= 1) {
      u = 1 - u;
      v = 1 - v;
    }

    auto const origin = (1 - u - v) * v0_ + u * v1_ + v * v2_;

    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = sampler->hemispherePSA(normal_);

    return first_ray_type(origin, direction_o, normal_);
  }
};

}
}
}
