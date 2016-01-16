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

#include "amber/scene/material.h"

namespace amber {
namespace scene {

struct BasicScatter
{
  UnitVector3 direction;
  real_type weight;

  BasicScatter(const UnitVector3& direction, real_type weight) noexcept;
};

template <typename Radiant, typename BasicMaterialForwardee>
class BasicMaterialForwarder
: public Material<Radiant>
{
public:
  BasicMaterialForwarder(
    const BasicMaterialForwardee& forwardee,
    const Radiant& rho
  ) noexcept;

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

private:
  BasicMaterialForwardee forwardee_;
  Radiant rho_;
};

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
class SymmetricBasicMaterialForwarder
: public Material<Radiant>
{
public:
  SymmetricBasicMaterialForwarder(
    const SymmetricBasicMaterialForwardee& forwardee,
    const Radiant& rho
  ) noexcept;

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

private:
  SymmetricBasicMaterialForwardee forwardee_;
  Radiant rho_;
};





inline
BasicScatter::BasicScatter(
  const UnitVector3& direction,
  real_type weight
) noexcept
: direction(direction)
, weight(weight)
{}

template <typename Radiant, typename BasicMaterialForwardee>
BasicMaterialForwarder<Radiant, BasicMaterialForwardee>
::BasicMaterialForwarder(
  const BasicMaterialForwardee& forwardee,
  const Radiant& rho
) noexcept
: forwardee_(forwardee)
, rho_(rho)
{}

template <typename Radiant, typename BasicMaterialForwardee>
rendering::SurfaceType
BasicMaterialForwarder<Radiant, BasicMaterialForwardee>
::Surface() const noexcept
{
  return forwardee_.Surface();
}

template <typename Radiant, typename BasicMaterialForwardee>
const Radiant
BasicMaterialForwarder<Radiant, BasicMaterialForwardee>
::BSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return rho_ * forwardee_.BSDF(normal, direction_out, direction_in);
}

template <typename Radiant, typename BasicMaterialForwardee>
const Radiant
BasicMaterialForwarder<Radiant, BasicMaterialForwardee>
::AdjointBSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return rho_ * forwardee_.AdjointBSDF(normal, direction_out, direction_in);
}

template <typename Radiant, typename BasicMaterialForwardee>
const real_type
BasicMaterialForwarder<Radiant, BasicMaterialForwardee>
::PDFLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return forwardee_.PDFLight(normal, direction_out, direction_in);
}

template <typename Radiant, typename BasicMaterialForwardee>
const real_type
BasicMaterialForwarder<Radiant, BasicMaterialForwardee>
::PDFImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return forwardee_.PDFImportance(normal, direction_out, direction_in);
}

template <typename Radiant, typename BasicMaterialForwardee>
rendering::Scatter<Radiant>
BasicMaterialForwarder<Radiant, BasicMaterialForwardee>
::SampleLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  const auto scatter =
    forwardee_.SampleLight(normal, direction_out, sampler);
  return rendering::Scatter<Radiant>(scatter.direction, scatter.weight * rho_);
}

template <typename Radiant, typename BasicMaterialForwardee>
rendering::Scatter<Radiant>
BasicMaterialForwarder<Radiant, BasicMaterialForwardee>
::SampleImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  const auto scatter =
    forwardee_.SampleImportance(normal, direction_out, sampler);
  return rendering::Scatter<Radiant>(scatter.direction, scatter.weight * rho_);
}

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
SymmetricBasicMaterialForwarder<Radiant, SymmetricBasicMaterialForwardee>
::SymmetricBasicMaterialForwarder(
  const SymmetricBasicMaterialForwardee& forwardee,
  const Radiant& rho
) noexcept
: forwardee_(forwardee)
, rho_(rho)
{}

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
rendering::SurfaceType
SymmetricBasicMaterialForwarder<Radiant, SymmetricBasicMaterialForwardee>
::Surface() const noexcept
{
  return forwardee_.Surface();
}

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
const Radiant
SymmetricBasicMaterialForwarder<Radiant, SymmetricBasicMaterialForwardee>
::BSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return rho_ * forwardee_.SymmetricBSDF(normal, direction_out, direction_in);
}

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
const Radiant
SymmetricBasicMaterialForwarder<Radiant, SymmetricBasicMaterialForwardee>
::AdjointBSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return rho_ * forwardee_.SymmetricBSDF(normal, direction_out, direction_in);
}

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
const real_type
SymmetricBasicMaterialForwarder<Radiant, SymmetricBasicMaterialForwardee>
::PDFLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return forwardee_.PDF(normal, direction_out, direction_in);
}

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
const real_type
SymmetricBasicMaterialForwarder<Radiant, SymmetricBasicMaterialForwardee>
::PDFImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return forwardee_.PDF(normal, direction_out, direction_in);
}

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
rendering::Scatter<Radiant>
SymmetricBasicMaterialForwarder<Radiant, SymmetricBasicMaterialForwardee>
::SampleLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  const auto scatter = forwardee_.Sample(normal, direction_out, sampler);
  return rendering::Scatter<Radiant>(scatter.direction, scatter.weight * rho_);
}

template <typename Radiant, typename SymmetricBasicMaterialForwardee>
rendering::Scatter<Radiant>
SymmetricBasicMaterialForwarder<Radiant, SymmetricBasicMaterialForwardee>
::SampleImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  const auto scatter = forwardee_.Sample(normal, direction_out, sampler);
  return rendering::Scatter<Radiant>(scatter.direction, scatter.weight * rho_);
}

}
}
