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

#include "amber/constants.h"
#include "amber/rendering/surface.h"
#include "amber/scene/material.h"

namespace amber {
namespace scene {

template <typename Radiant>
std::unique_ptr<Material<Radiant>>
MakeDiffuseLight(const Radiant& radiance);

template <typename Radiant>
class DiffuseLight
: public Material<Radiant>
{
public:
  explicit DiffuseLight(const Radiant& radiance) noexcept;

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

  const real_type
  PDFLight(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept;

  const real_type
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
  Radiant radiance_;
};





template <typename Radiant>
std::unique_ptr<Material<Radiant>>
MakeDiffuseLight(const Radiant& radiance)
{
  return std::make_unique<DiffuseLight<Radiant>>(radiance);
}

template <typename Radiant>
DiffuseLight<Radiant>::DiffuseLight(const Radiant& radiance) noexcept
: radiance_(radiance)
{}

template <typename Radiant>
rendering::SurfaceType
DiffuseLight<Radiant>::Surface() const noexcept
{
  return rendering::SurfaceType::Light;
}

template <typename Radiant>
const Radiant
DiffuseLight<Radiant>::Irradiance() const noexcept
{
  return radiance_ * static_cast<real_type>(kPI);
}

template <typename Radiant>
const Radiant
DiffuseLight<Radiant>::Radiance(
  const UnitVector3& normal,
  const UnitVector3& direction_out
) const noexcept
{
  if (Dot(direction_out, normal) <= 0) {
    return Radiant();
  }

  return radiance_;
}

template <typename Radiant>
const Radiant
DiffuseLight<Radiant>::BSDF(
  const UnitVector3&,
  const UnitVector3&,
  const UnitVector3&
) const noexcept
{
  return Radiant();
}

template <typename Radiant>
const Radiant
DiffuseLight<Radiant>::AdjointBSDF(
  const UnitVector3&,
  const UnitVector3&,
  const UnitVector3&
) const noexcept
{
  return Radiant();
}

template <typename Radiant>
const real_type
DiffuseLight<Radiant>::PDFLight(
  const UnitVector3&,
  const UnitVector3&,
  const UnitVector3&
) const noexcept
{
  return kDiracDelta;
}

template <typename Radiant>
const real_type
DiffuseLight<Radiant>::PDFImportance(
  const UnitVector3&,
  const UnitVector3&,
  const UnitVector3&
) const noexcept
{
  return kDiracDelta;
}

template <typename Radiant>
rendering::Scatter<Radiant>
DiffuseLight<Radiant>::SampleLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return rendering::Scatter<Radiant>();
}

template <typename Radiant>
rendering::Scatter<Radiant>
DiffuseLight<Radiant>::SampleImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return rendering::Scatter<Radiant>();
}

}
}
