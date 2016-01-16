// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include <algorithm>

#include "amber/prelude/aabb.h"
#include "amber/prelude/hit.h"
#include "amber/prelude/ray.h"
#include "amber/prelude/sampling.h"
#include "amber/prelude/vector3.h"
#include "amber/scene/primitive_triangle.h"

namespace amber {
namespace scene {

class Triangle
: public Primitive
{
public:
  Triangle(const Vector3& v0, const Vector3& v1, const Vector3& v2) noexcept;

  const Vector3 Center() const noexcept;
  const AABB BoundingBox() const noexcept;
  Hit Intersect(const Ray& ray) const noexcept;

  const real_type SurfaceArea() const noexcept;
  Ray SampleSurfacePoint(Sampler& sampler) const;

private:
  Vector3 v0_, v1_, v2_;
  UnitVector3 normal_;
};



std::unique_ptr<Primitive>
MakeTriangle(const Vector3& v0, const Vector3& v1, const Vector3& v2) noexcept
{
  return std::make_unique<Triangle>(v0, v1, v2);
}

Triangle::Triangle(
  const Vector3& v0,
  const Vector3& v1,
  const Vector3& v2
) noexcept
: v0_(v0)
, v1_(v1)
, v2_(v2)
, normal_(Normalize(Cross(v1 - v0, v2 - v0)))
{}

const Vector3
Triangle::Center() const noexcept
{
  return Vector3(
    (v0_.X() + v1_.X() + v2_.X()) / 3,
    (v0_.Y() + v1_.Y() + v2_.Y()) / 3,
    (v0_.Z() + v1_.Z() + v2_.Z()) / 3
  );
}

const AABB
Triangle::BoundingBox() const noexcept
{
  return AABB(
    Vector3(
      std::min({v0_.X(), v1_.X(), v2_.X()}),
      std::min({v0_.Y(), v1_.Y(), v2_.Y()}),
      std::min({v0_.Z(), v1_.Z(), v2_.Z()})
    ),
    Vector3(
      std::max({v0_.X(), v1_.X(), v2_.X()}),
      std::max({v0_.Y(), v1_.Y(), v2_.Y()}),
      std::max({v0_.Z(), v1_.Z(), v2_.Z()})
    )
  );
}

Hit
Triangle::Intersect(const Ray& ray) const noexcept
{
  const auto E1 = v1_ - v0_;
  const auto E2 = v2_ - v0_;

  const auto P = Cross(ray.direction, E2);
  const auto det = Dot(P, E1);

  const auto T = ray.origin - v0_;
  const auto u = Dot(P, T) / det;
  if (u > 1 || u < 0) {
    return Hit();
  }

  const auto Q = Cross(T, E1);
  const auto v = Dot(Q, ray.direction) / det;
  if (v > 1 || v < 0) {
    return Hit();
  }

  if (u + v > 1) {
    return Hit();
  }

  const auto t = Dot(Q, E2) / det;
  if (t < kEPS) {
    return Hit();
  }

  return Hit(v0_ + u * E1 + v * E2, normal_, t);
}

const real_type
Triangle::SurfaceArea() const noexcept
{
  return Length(Cross(v1_ - v0_, v2_ - v0_)) / 2;
}

Ray
Triangle::SampleSurfacePoint(Sampler& sampler) const
{
  auto u = prelude::Uniform<real_type>(sampler);
  auto v = prelude::Uniform<real_type>(sampler);

  if (u + v >= 1) {
    u = 1 - u;
    v = 1 - v;
  }

  const auto origin = (1 - u - v) * v0_ + u * v1_ + v * v2_;

  return Ray(origin, normal_);
}

}
}
