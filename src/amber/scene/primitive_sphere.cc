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
#include "amber/scene/primitive_sphere.h"

namespace amber {
namespace scene {

class Sphere
: public Primitive
{
public:
  Sphere(const Vector3& center, real_type radius) noexcept;

  const Vector3 Center() const noexcept;
  const AABB BoundingBox() const noexcept;
  Hit Intersect(const Ray& ray) const noexcept;

  const real_type SurfaceArea() const noexcept;
  Ray SampleSurfacePoint(Sampler& sampler) const;

private:
  Vector3 center_;
  real_type radius_;
};



std::unique_ptr<Primitive>
MakeSphere(const Vector3& center, real_type radius) noexcept
{
  return std::make_unique<Sphere>(center, radius);
}

Sphere::Sphere(const Vector3& center, const real_type radius) noexcept
: center_(center)
, radius_(radius)
{}

const Vector3
Sphere::Center() const noexcept
{
  return center_;
}

const AABB
Sphere::BoundingBox() const noexcept
{
  return AABB(center_ - radius_, center_ + radius_);
}

Hit
Sphere::Intersect(const Ray& ray) const noexcept
{
  const auto a = static_cast<real_type>(1);
  const auto b = -2 * Dot(center_ - ray.origin, ray.direction);
  const auto c = SquaredLength(center_ - ray.origin) - radius_ * radius_;

  const auto solutions = prelude::SolveQuadratic(a, b, c);
  if (!solutions) {
    return Hit();
  }

  const auto& alpha = std::get<0>(*solutions);
  const auto& beta = std::get<1>(*solutions);

  if (alpha > kEPS) {
    return Hit(
      ray.origin + alpha * ray.direction,
      ray.origin + alpha * ray.direction - center_,
      alpha
    );
  }

  if (beta > kEPS) {
    return Hit(
      ray.origin + beta * ray.direction,
      ray.origin + beta * ray.direction - center_,
      beta
    );
  }

  return Hit();
}

const real_type
Sphere::SurfaceArea() const noexcept
{
  return 4 * static_cast<real_type>(kPI) * radius_ * radius_;
}

Ray
Sphere::SampleSurfacePoint(Sampler& sampler) const
{
  const auto normal = prelude::SphereSA<real_type>(sampler);
  const auto origin = center_ + radius_ * normal;

  return Ray(origin, normal);
}

}
}
