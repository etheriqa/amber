// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "core/algebra.h"
#include "core/constant.h"
#include "core/primitive/disk.h"

namespace amber {
namespace core {
namespace primitive {

template <typename RealType>
class Cylinder : public Primitive<RealType>
{
public:
  using vector3_type = Vector3<RealType>;

private:
  using typename Primitive<RealType>::aabb_type;
  using typename Primitive<RealType>::hit_type;
  using typename Primitive<RealType>::ray_type;

  using disk_type = Disk<RealType>;

  vector3_type center_, normal_;
  RealType radius_, height_;

public:
  Cylinder(
    vector3_type const& center,
    vector3_type const& normal,
    RealType radius,
    RealType height
  ) noexcept
  : center_(center),
    normal_(Normalize(normal)),
    radius_(radius),
    height_(height)
  {}

  RealType SurfaceArea() const noexcept
  {
    return 2 * static_cast<RealType>(kPI) * radius_ * height_;
  }

  aabb_type BoundingBox() const noexcept
  {
    auto const bottom = disk_type(center_, normal_, radius_);
    auto const top = disk_type(center_ + height_ * normal_, normal_, radius_);
    return bottom.BoundingBox() + top.BoundingBox();
  }

  hit_type Intersect(ray_type const& ray) const noexcept
  {
    auto const OC = center_ - ray.origin;
    auto const u = ray.direction - Dot(ray.direction, normal_) * normal_;
    auto const v = OC - Dot(OC, normal_) * normal_;

    auto const a = SquaredLength(u);
    auto const b = -2 * Dot(u, v);
    auto const c = SquaredLength(v) - radius_ * radius_;

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
}
