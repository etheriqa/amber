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
#include "primitive/disk.h"

namespace amber {
namespace primitive {

template <typename RealType>
class Cylinder : public Primitive<RealType>
{
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
  using hit_type       = typename Primitive<RealType>::hit_type;
  using ray_type       = typename Primitive<RealType>::ray_type;

  using vector3_type   = Vector3<RealType>;

private:
  using disk_type      = Disk<RealType>;

  vector3_type center_, normal_;
  RealType radius_, height_;

public:
  Cylinder(vector3_type const& center,
           vector3_type const& normal,
           RealType radius,
           RealType height) noexcept
    : center_(center),
      normal_(Normalize(normal)),
      radius_(radius),
      height_(height) {}

  RealType SurfaceArea() const noexcept {
    return 2 * static_cast<RealType>(kPI) * radius_ * height_;
  }

  aabb_type BoundingBox() const noexcept {
    auto const bottom = disk_type(center_, normal_, radius_);
    auto const top = disk_type(center_ + height_ * normal_, normal_, radius_);
    return bottom.BoundingBox() + top.BoundingBox();
  }

  hit_type Intersect(ray_type const& ray) const noexcept {
    auto const OC = center_ - ray.origin;
    auto const u = ray.direction - Dot(ray.direction, normal_) * normal_;
    auto const v = OC - Dot(OC, normal_) * normal_;

    auto const a = u.SquaredLength();
    auto const b = -2 * Dot(u, v);
    auto const c = v.SquaredLength() - radius_ * radius_;

    auto const solutions = SolveQuadratic(a, b, c);
    if (!solutions) {
      return hit_type();
    }

    auto const& alpha = std::get<0>(*solutions);
    auto const& beta = std::get<1>(*solutions);

    if (alpha > kEPS) {
      auto const h = Dot(alpha * ray.direction - OC, normal_);
      if (h >= 0 && h <= height_) {
        return
          hit_type(ray.origin + alpha * ray.direction,
                   ray.origin + alpha * ray.direction - center_ - h * normal_,
                   alpha);
      }
    }

    if (beta > kEPS) {
      auto const h = Dot(beta * ray.direction - OC, normal_);
      if (h >= 0 && h <= height_) {
        return
          hit_type(ray.origin + beta * ray.direction,
                   ray.origin + beta * ray.direction - center_ - h * normal_,
                   beta);
      }
    }

    return hit_type();
  }

  ray_type SamplePoint(Sampler* sampler) const
  {
    auto const height = sampler->uniform(height_);

    vector3_type u, v;
    std::tie(u, v) = OrthonormalBasis(normal_);

    RealType x, y;
    std::tie(x, y) = sampler->circle<RealType>();

    auto const origin = center_ + normal_ * height + normal * radius_;
    auto const normal = u * x + v * y;

    return ray_type(origin, normal);
  }
};

}
}