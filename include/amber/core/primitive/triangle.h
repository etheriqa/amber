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

#include <algorithm>

#include "core/constant.h"
#include "core/primitive.h"

namespace amber {
namespace core {
namespace primitive {

template <typename RealType>
class Triangle : public Primitive<RealType>
{
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
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
    : v0_(v0), v1_(v1), v2_(v2), normal_(Normalize(Cross(v1 - v0, v2 - v0))) {}

  RealType SurfaceArea() const noexcept {
    return (Cross(v1_ - v0_, v2_ - v0_)).Length() / 2;
  }

  aabb_type BoundingBox() const noexcept {
    return aabb_type(vector3_type(std::min({v0_.x(), v1_.x(), v2_.x()}),
                                  std::min({v0_.y(), v1_.y(), v2_.y()}),
                                  std::min({v0_.z(), v1_.z(), v2_.z()})),
                     vector3_type(std::max({v0_.x(), v1_.x(), v2_.x()}),
                                  std::max({v0_.y(), v1_.y(), v2_.y()}),
                                  std::max({v0_.z(), v1_.z(), v2_.z()})));
  }

  hit_type Intersect(ray_type const& ray) const noexcept {
    auto const E1 = v1_ - v0_;
    auto const E2 = v2_ - v0_;

    auto const P = Cross(ray.direction, E2);
    auto const det = Dot(P, E1);

    auto const T = ray.origin - v0_;
    auto const u = Dot(P, T) / det;
    if (u > 1 || u < 0) {
      return hit_type();
    }

    auto const Q = Cross(T, E1);
    auto const v = Dot(Q, ray.direction) / det;
    if (v > 1 || v < 0) {
      return hit_type();
    }

    if (u + v > 1) {
      return hit_type();
    }

    auto const t = Dot(Q, E2) / det;
    if (t < kEPS) {
      return hit_type();
    }

    return hit_type(v0_ + u * E1 + v * E2,
                    normal_,
                    t);
  }

  ray_type SamplePoint(Sampler* sampler) const
  {
    auto u = sampler->uniform<RealType>();
    auto v = sampler->uniform<RealType>();

    if (u + v >= 1) {
      u = 1 - u;
      v = 1 - v;
    }

    auto const origin = (1 - u - v) * v0_ + u * v1_ + v * v2_;

    return ray_type(origin, normal_);
  }
};

}
}
}
