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
#include "core/primitive.h"

namespace amber {
namespace core {
namespace primitive {

template <typename RealType>
class Sphere : public Primitive<RealType>
{
public:
  using vector3_type = Vector3<RealType>;

private:
  using typename Primitive<RealType>::aabb_type;
  using typename Primitive<RealType>::hit_type;
  using typename Primitive<RealType>::ray_type;

  vector3_type center_;
  RealType radius_;

public:
  Sphere(vector3_type const& center, RealType radius) noexcept
  : center_(center), radius_(radius) {}

  RealType SurfaceArea() const noexcept
  {
    return 4 * static_cast<RealType>(kPI) * radius_ * radius_;
  }

  aabb_type BoundingBox() const noexcept
  {
    return aabb_type(center_ - radius_, center_ + radius_);
  }

  hit_type Intersect(ray_type const& ray) const noexcept
  {
    auto const a = static_cast<RealType>(1);
    auto const b = -2 * Dot(center_ - ray.origin, ray.direction);
    auto const c = SquaredLength(center_ - ray.origin) - radius_ * radius_;

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
}
