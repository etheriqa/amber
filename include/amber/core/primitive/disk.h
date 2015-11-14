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

#include <cmath>

#include "core/constant.h"
#include "core/primitive.h"

namespace amber {
namespace core {
namespace primitive {

template <typename RealType>
class Disk : public Primitive<RealType>
{
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
  using hit_type       = typename Primitive<RealType>::hit_type;
  using ray_type       = typename Primitive<RealType>::ray_type;

  using vector3_type   = Vector3<RealType>;

private:
  vector3_type center_, normal_;
  RealType radius_;

public:
  Disk(vector3_type const& center,
       vector3_type const& normal,
       RealType radius) noexcept
    : center_(center), normal_(Normalize(normal)), radius_(radius) {}

  RealType SurfaceArea() const noexcept {
    return static_cast<RealType>(kPI) * radius_ * radius_;
  }

  aabb_type BoundingBox() const noexcept {
    auto const factor =
      vector3_type(std::sqrt(1 - normal_.x() * normal_.x()),
                   std::sqrt(1 - normal_.y() * normal_.y()),
                   std::sqrt(1 - normal_.z() * normal_.z()));

    return aabb_type(center_ - radius_ * factor, center_ + radius_ * factor);
  }

  hit_type Intersect(ray_type const& ray) const noexcept {
     auto const cos_theta = Dot(ray.direction, normal_);
     if (cos_theta == 0) {
       return hit_type();
     }

     auto const t = Dot(center_ - ray.origin, normal_) / cos_theta;
     if (t < kEPS) {
       return hit_type();
     }

     auto const squared_distance =
       SquaredLength(ray.origin + t * ray.direction - center_);
     if (squared_distance > radius_ * radius_) {
       return hit_type();
     }

     return hit_type(ray.origin + t * ray.direction, normal_, t);
  }

  ray_type SamplePoint(Sampler* sampler) const
  {
    auto const radius = std::sqrt(sampler->uniform(radius_ * radius_));

    vector3_type u, v;
    std::tie(u, v) = OrthonormalBasis(normal_);

    RealType x, y;
    std::tie(x, y) = sampler->circle<RealType>();

    auto const origin = center_ + (u * x + v * y) * radius;

    return ray_type(origin, normal_);
  }
};

}
}
}
