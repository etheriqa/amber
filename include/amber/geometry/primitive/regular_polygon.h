/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>

#include "constant.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class RegularPolygon : public Primitive<RealType> {
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
  using hit_type       = typename Primitive<RealType>::hit_type;
  using first_ray_type = typename Primitive<RealType>::first_ray_type;
  using ray_type       = typename Primitive<RealType>::ray_type;

  using vector3_type   = Vector3<RealType>;

private:
  vector3_type center_, u_, v_, w_;
  size_t n_;
  RealType angle_, long_radius_, short_radius_, surface_area_;
  aabb_type aabb_;

public:
  RegularPolygon(
    vector3_type const& center,
    vector3_type const& direction,
    vector3_type const& up,
    size_t n,
    RealType radius
  ) noexcept
  : center_(center),
    n_(n),
    angle_(2 * kPI / n_),
    long_radius_(radius),
    short_radius_(long_radius_ * std::cos(angle_ / 2)),
    surface_area_(n * radius * radius * std::sin(2 * kPI / n_)),
    aabb_()
  {
    w_ = normalize(direction);
    u_ = normalize(cross(up, w_));
    v_ = normalize(cross(w_, u_));
    for (size_t i = 0; i < n; i++) {
      auto const theta = angle_ * i;
      aabb_ += aabb_type(
        center_ + long_radius_ * (u_ * std::sin(theta) + v_ * std::cos(theta))
      );
    }
  }

  RealType surfaceArea() const noexcept
  {
    return surface_area_;
  }

  aabb_type aabb() const noexcept
  {
    return aabb_;
  }

  hit_type intersect(ray_type const& ray) const noexcept
  {
    auto const P = cross(ray.direction, v_);
    auto const det = dot(P, u_);

    auto const T = ray.origin - center_;
    auto const x = dot(P, T) / det;
    if (x < -long_radius_ || x > long_radius_) {
      return hit_type();
    }

    auto const Q = cross(T, u_);
    auto const y = dot(Q, ray.direction) / det;
    if (y < -long_radius_ || y > long_radius_) {
      return hit_type();
    }

    auto const t = dot(Q, v_) / det;
    if (t < kEPS) {
      return hit_type();
    }

    auto const distance = std::hypot(x, y);
    auto const theta =
      fmod(
        std::atan2(y, x) + static_cast<RealType>(1.5 * kPI) + angle_ / 2,
        angle_
      ) - angle_ / 2;
    if (distance * std::cos(theta) > short_radius_) {
      return hit_type();
    }

    return hit_type(center_ + x * u_ + y * v_, w_, t);
  }

  first_ray_type sampleFirstRay(Sampler* sampler) const
  {
    auto const x = std::sqrt(sampler->uniform(short_radius_ * short_radius_));
    auto const y = x * std::tan(angle_ / 2) * sampler->uniform<RealType>(-1, 1);
    auto const theta =
      angle_ * std::floor(sampler->uniform<RealType>(n_)) +
      static_cast<RealType>(kPI / 2);

    return first_ray_type(
      center_ +
        u_ * (x * std::cos(theta) - y * std::sin(theta)) +
        v_ * (x * std::sin(theta) + y * std::cos(theta)),
      std::get<0>(sampler->hemispherePSA(w_)),
      w_
    );
  }
};

}
}
}
