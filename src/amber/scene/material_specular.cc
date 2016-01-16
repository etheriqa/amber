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

#include "amber/constants.h"
#include "amber/prelude/geometry.h"
#include "amber/rendering/surface.h"
#include "amber/scene/material_specular.h"

namespace amber {
namespace scene {

rendering::SurfaceType
BasicSpecular::Surface() const noexcept
{
  return rendering::SurfaceType::Specular;
}

const real_type
BasicSpecular::SymmetricBSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  const auto signed_cos_o = Dot(direction_out, normal);
  const auto signed_cos_i = Dot(direction_in, normal);

  if (signed_cos_o * signed_cos_i <= 0) {
    return 0;
  }

  return kDiracDelta;
}

const real_type
BasicSpecular::PDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  return kDiracDelta;
}

BasicScatter
BasicSpecular::Sample(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return BasicScatter(prelude::PerfectReflection(direction_out, normal), 1);
}

}
}
