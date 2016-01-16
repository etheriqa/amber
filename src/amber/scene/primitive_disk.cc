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
#include "amber/prelude/hit.h"
#include "amber/prelude/ray.h"
#include "amber/prelude/sampling.h"
#include "amber/prelude/vector3.h"
#include "amber/scene/primitive_disk.h"

namespace amber {
namespace scene {

class Disk
: public Primitive
{
public:
  Disk(
    const Vector3& center,
    const Vector3& normal,
    real_type radius
  ) noexcept;

  const Vector3 Center() const noexcept;
  const AABB BoundingBox() const noexcept;
  Hit Intersect(const Ray& ray) const noexcept;

  const real_type SurfaceArea() const noexcept;
  Ray SampleSurfacePoint(Sampler& sampler) const;

private:
  Vector3 center_;
  UnitVector3 normal_;
  real_type radius_;
};



std::unique_ptr<Primitive>
MakeDisk(
  const Vector3& center,
  const Vector3& normal,
  real_type radius
) noexcept
{
  return std::make_unique<Disk>(center, normal, radius);
}

Disk::Disk(
  const Vector3& center,
  const Vector3& normal,
  real_type radius
) noexcept
: center_(center)
, normal_(normal)
, radius_(radius)
{}

const Vector3
Disk::Center() const noexcept
{
  return center_;
}

const AABB
Disk::BoundingBox() const noexcept
{
  const auto factor = Vector3(
    std::sqrt(1 - normal_.X() * normal_.X()),
    std::sqrt(1 - normal_.Y() * normal_.Y()),
    std::sqrt(1 - normal_.Z() * normal_.Z())
  );

  return AABB(center_ - radius_ * factor, center_ + radius_ * factor);
}

Hit
Disk::Intersect(const Ray& ray) const noexcept
{
   const auto cos_theta = Dot(ray.direction, normal_);
   if (cos_theta == 0) {
     return Hit();
   }

   const auto t = Dot(center_ - ray.origin, normal_) / cos_theta;
   if (t < kEPS) {
     return Hit();
   }

   const auto squared_distance =
     SquaredLength(ray.origin + t * ray.direction - center_);
   if (squared_distance > radius_ * radius_) {
     return Hit();
   }

   return Hit(ray.origin + t * ray.direction, normal_, t);
}

const real_type
Disk::SurfaceArea() const noexcept
{
  return static_cast<real_type>(kPI) * radius_ * radius_;
}

Ray
Disk::SampleSurfacePoint(Sampler& sampler) const
{
  const auto radius =
    std::sqrt(prelude::Uniform(radius_ * radius_, sampler));

  Vector3 u, v;
  std::tie(u, v) = OrthonormalBasis(normal_);

  const auto angle = prelude::Circle<real_type>(sampler);

  const auto origin = center_ + (u * angle.X() + v * angle.Y()) * radius;

  return Ray(origin, normal_);
}

}
}
