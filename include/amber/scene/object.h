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

#pragma once

#include "amber/prelude/aabb.h"
#include "amber/prelude/hit.h"
#include "amber/prelude/ray.h"
#include "amber/rendering/scatter.h"
#include "amber/scene/material.h"
#include "amber/scene/primitive.h"

namespace amber {
namespace scene {

template <typename Radiant>
class Object
{
public:
  Object() noexcept;
  Object(
    const Primitive* primitive,
    const Material<Radiant>* material
  ) noexcept;

  const Vector3 Center() const noexcept;
  const AABB BoundingBox() const noexcept;
  Hit Intersect(const Ray& ray) const noexcept;
  real_type SurfaceArea() const noexcept;
  Ray SampleSurfacePoint(Sampler& sampler) const;

  rendering::SurfaceType Surface() const noexcept;
  const Radiant Irradiance() const noexcept;
  const Radiant
  Radiance(
    const UnitVector3& normal,
    const UnitVector3& direction_out
  ) const noexcept;
  const Radiant
  BSDF(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept;
  const Radiant
  AdjointBSDF(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept;
  real_type
  PDFLight(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept;
  real_type
  PDFImportance(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept;
  rendering::Scatter<Radiant>
  SampleLight(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const;
  rendering::Scatter<Radiant>
  SampleImportance(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const;

private:
  const Primitive* primitive_;
  const Material<Radiant>* material_;
};

template <typename Radiant>
real_type
Power(const Object<Radiant>& object) noexcept
{
  return Sum(object.SurfaceArea() * object.Irradiance());
}



template <typename Radiant>
Object<Radiant>::Object() noexcept
: primitive_(nullptr)
, material_(nullptr)
{}

template <typename Radiant>
Object<Radiant>::Object(
  const Primitive* primitive,
  const Material<Radiant>* material
) noexcept
: primitive_(primitive)
, material_(material)
{}

template <typename Radiant>
const Vector3
Object<Radiant>::Center() const noexcept
{
  return primitive_->Center();
}

template <typename Radiant>
const AABB
Object<Radiant>::BoundingBox() const noexcept
{
  return primitive_->BoundingBox();
}

template <typename Radiant>
Hit
Object<Radiant>::Intersect(const Ray& ray) const noexcept
{
  return primitive_->Intersect(ray);
}

template <typename Radiant>
real_type
Object<Radiant>::SurfaceArea() const noexcept
{
  return primitive_->SurfaceArea();
}

template <typename Radiant>
Ray
Object<Radiant>::SampleSurfacePoint(Sampler& sampler) const
{
  return primitive_->SampleSurfacePoint(sampler);
}

template <typename Radiant>
rendering::SurfaceType
Object<Radiant>::Surface() const noexcept
{
  return material_->Surface();
}

template <typename Radiant>
const Radiant
Object<Radiant>::Irradiance() const noexcept
{
  return material_->Irradiance();
}

template <typename Radiant>
const Radiant
Object<Radiant>::Radiance(
  const UnitVector3& normal,
  const UnitVector3& direction_out
) const noexcept
{
  return material_->Radiance(normal, direction_out);
}

template <typename Radiant>
const Radiant
Object<Radiant>::BSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return material_->BSDF(normal, direction_out, direction_in);
}

template <typename Radiant>
const Radiant
Object<Radiant>::AdjointBSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return material_->AdjointBSDF(normal, direction_out, direction_in);
}

template <typename Radiant>
real_type
Object<Radiant>::PDFLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return material_->PDFLight(normal, direction_out, direction_in);
}

template <typename Radiant>
real_type
Object<Radiant>::PDFImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return material_->PDFImportance(normal, direction_out, direction_in);
}

template <typename Radiant>
rendering::Scatter<Radiant>
Object<Radiant>::SampleLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return material_->SampleLight(normal, direction_out, sampler);
}

template <typename Radiant>
rendering::Scatter<Radiant>
Object<Radiant>::SampleImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return material_->SampleImportance(normal, direction_out, sampler);
}

}
}
