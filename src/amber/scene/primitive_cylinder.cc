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

#include "amber/prelude/aabb.h"
#include "amber/prelude/algebra.h"
#include "amber/prelude/hit.h"
#include "amber/prelude/ray.h"
#include "amber/prelude/sampling.h"
#include "amber/prelude/vector3.h"
#include "amber/scene/primitive_cylinder.h"
#include "amber/scene/primitive_disk.h"

namespace amber {
namespace scene {

class Cylinder
: public Primitive
{
public:
  Cylinder(
    const Vector3& center,
    const Vector3& normal,
    real_type radius,
    real_type height
  ) noexcept;

  const Vector3 Center() const noexcept;
  const AABB BoundingBox() const noexcept;
  Hit Intersect(const Ray& ray) const noexcept;

  real_type SurfaceArea() const noexcept;
  Ray SampleSurfacePoint(Sampler& sampler) const;

private:
  Vector3 center_;
  UnitVector3 normal_;
  real_type radius_;
  real_type height_;
  AABB bb_;
};



std::unique_ptr<Primitive>
MakeCylinder(
  const Vector3& center,
  const Vector3& normal,
  real_type radius,
  real_type height
) noexcept
{
  return std::make_unique<Cylinder>(center, normal, radius, height);
}

Cylinder::Cylinder(
  const Vector3& center,
  const Vector3& normal,
  real_type radius,
  real_type height
) noexcept
: center_(center)
, normal_(normal)
, radius_(radius)
, height_(height)
, bb_(AABB::Empty())
{
  bb_ += MakeDisk(center_, normal_, radius_)->BoundingBox();
  bb_ += MakeDisk(center_ + height_ * normal_, normal_, radius_)->BoundingBox();
}

const Vector3
Cylinder::Center() const noexcept
{
  return center_ + height_ / 2 * normal_;
}

const AABB
Cylinder::BoundingBox() const noexcept
{
  return bb_;
}

Hit
Cylinder::Intersect(const Ray& ray) const noexcept
{
  const auto OC = center_ - ray.Origin();
  const auto u = ray.Direction() - Dot(ray.Direction(), normal_) * normal_;
  const auto v = OC - Dot(OC, normal_) * normal_;

  const auto a = SquaredLength(u);
  const auto b = -2 * Dot(u, v);
  const auto c = SquaredLength(v) - radius_ * radius_;

  const auto solutions = prelude::SolveQuadratic(a, b, c);
  if (!solutions) {
    return Hit();
  }

  const auto& alpha = std::get<0>(*solutions);
  const auto& beta = std::get<1>(*solutions);

  if (alpha > kEPS) {
    const auto h = Dot(alpha * ray.Direction() - OC, normal_);
    if (h >= 0 && h <= height_) {
      return Hit(
        ray.Origin() + alpha * ray.Direction(),
        ray.Origin() + alpha * ray.Direction() - center_ - h * normal_,
        alpha
      );
    }
  }

  if (beta > kEPS) {
    const auto h = Dot(beta * ray.Direction() - OC, normal_);
    if (h >= 0 && h <= height_) {
      return Hit(
        ray.Origin() + beta * ray.Direction(),
        ray.Origin() + beta * ray.Direction() - center_ - h * normal_,
        beta
      );
    }
  }

  return Hit();
}

real_type
Cylinder::SurfaceArea() const noexcept
{
  return 2 * static_cast<real_type>(kPI) * radius_ * height_;
}

Ray
Cylinder::SampleSurfacePoint(Sampler& sampler) const
{
  const auto height = prelude::Uniform(height_, sampler);

  Vector3 u, v;
  std::tie(u, v) = OrthonormalBasis(normal_);

  const auto angle = prelude::Circle<real_type>(sampler);

  const auto normal = u * angle.X() + v * angle.Y();
  const auto origin = center_ + normal_ * height + normal * radius_;

  return Ray(origin, normal);
}

}
}
