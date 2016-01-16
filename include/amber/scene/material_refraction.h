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

#include <memory>

#include "amber/scene/material_basic.h"

namespace amber {
namespace scene {

template <typename Radiant>
std::unique_ptr<Material<Radiant>>
MakeRefraction(real_type ior);

class BasicRefraction
{
public:
  explicit BasicRefraction(real_type ior) noexcept;

  rendering::SurfaceType Surface() const noexcept;

  const real_type
  BSDF(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const noexcept;

  const real_type
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

  BasicScatter
  SampleLight(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const;

  BasicScatter
  SampleImportance(
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const;

private:
  real_type ior_, r0_;

  static const real_type Fresnel(real_type ior) noexcept;
  static const real_type Schlick(real_type r0, real_type cos_alpha) noexcept;
};



template <typename Radiant>
std::unique_ptr<Material<Radiant>>
MakeRefraction(real_type ior)
{
  return std::make_unique<BasicMaterialForwarder<Radiant, BasicRefraction>>(
    BasicRefraction(ior),
    Radiant(1)
  );
}

}
}
