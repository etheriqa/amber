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

#include "amber/rendering/forward.h"
#include "amber/scene/forward.h"

namespace amber {
namespace scene {

/** Material interface.
 */
template <typename Radiant>
class Material
{
public:
  virtual ~Material() {}

  virtual rendering::SurfaceType Surface() const noexcept = 0;

  virtual const Radiant Irradiance() const noexcept;

  virtual const Radiant
  Radiance(
    const UnitVector3& normal,
    const UnitVector3& direction_out
  ) const noexcept;

  virtual const Radiant
  BSDF(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept = 0;

  virtual const Radiant
  AdjointBSDF(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept = 0;

  virtual const real_type
  PDFLight(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept = 0;

  virtual const real_type
  PDFImportance(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept = 0;

  virtual rendering::Scatter<Radiant>
  SampleLight(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const = 0;

  virtual rendering::Scatter<Radiant>
  SampleImportance(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const = 0;
};





template <typename Radiant>
const Radiant
Material<Radiant>::Irradiance() const noexcept
{
  return Radiant();
}

template <typename Radiant>
const Radiant
Material<Radiant>::Radiance(
  const UnitVector3& normal,
  const UnitVector3& direction_out
) const noexcept
{
  return Radiant();
}

}
}
