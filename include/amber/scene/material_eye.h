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
MakeEye();

template <typename Radiant>
class Eye
: public Material<Radiant>
{
public:
  rendering::SurfaceType Surface() const noexcept;

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
};





template <typename Radiant>
std::unique_ptr<Material<Radiant>>
MakeEye()
{
  return std::make_unique<Eye<Radiant>>();
}

template <typename Radiant>
rendering::SurfaceType
Eye<Radiant>::Surface() const noexcept
{
  return rendering::SurfaceType::Eye;
}

template <typename Radiant>
const Radiant
Eye<Radiant>::BSDF(
  const UnitVector3&,
  const UnitVector3&,
  const UnitVector3&
) const noexcept
{
  return Radiant(kDiracDelta);
}

template <typename Radiant>
const Radiant
Eye<Radiant>::AdjointBSDF(
  const UnitVector3&,
  const UnitVector3&,
  const UnitVector3&
) const noexcept
{
  return Radiant(kDiracDelta);
}

template <typename Radiant>
const real_type
Eye<Radiant>::PDFLight(
  const UnitVector3&,
  const UnitVector3&,
  const UnitVector3&
) const noexcept
{
  return kDiracDelta;
}

template <typename Radiant>
const real_type
Eye<Radiant>::PDFImportance(
  const UnitVector3&,
  const UnitVector3&,
  const UnitVector3&
) const noexcept
{
  return kDiracDelta;
}

template <typename Radiant>
rendering::Scatter<Radiant>
Eye<Radiant>::SampleLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return rendering::Scatter<Radiant>(-direction_out, Radiant(1));
}

template <typename Radiant>
rendering::Scatter<Radiant>
Eye<Radiant>::SampleImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return rendering::Scatter<Radiant>(-direction_out, Radiant(1));
}

}
}
