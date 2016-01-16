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

#include "amber/prelude/geometry.h"
#include "amber/prelude/sampling.h"
#include "amber/rendering/surface.h"
#include "amber/scene/material_phong.h"

namespace amber {
namespace scene {

BasicPhong::BasicPhong(real_type exponent) noexcept
: exponent_(exponent)
{}

rendering::SurfaceType
BasicPhong::Surface() const noexcept
{
  return rendering::SurfaceType::Diffuse;
}

const real_type
BasicPhong::SymmetricBSDF(
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

  const auto direction_reflection =
    prelude::PerfectReflection(direction_in, normal, signed_cos_i);
  const auto cos_alpha =
    std::max<real_type>(0, Dot(direction_reflection, direction_out));

  return
    (exponent_ + 2) / (2 * static_cast<real_type>(kPI)) *
    std::pow(cos_alpha, exponent_);
}

const real_type
BasicPhong::PDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  const auto direction_reflection =
    prelude::PerfectReflection(direction_out, normal);
  const auto cos_alpha =
    std::max<real_type>(0, Dot(direction_in, direction_reflection));
  const auto cos_i = std::abs(Dot(direction_in, normal));

  return
    (exponent_ + 1) / (2 * static_cast<real_type>(kPI)) *
    std::pow(cos_alpha, exponent_) / cos_i;
}

BasicScatter
BasicPhong::Sample(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  const auto direction_reflection =
    prelude::PerfectReflection(direction_out, normal);
  const auto signed_cos_o = Dot(direction_out, normal);

  for (;;) {
    const auto direction_in =
      prelude::CosinePower(direction_reflection, exponent_, sampler);
    const auto signed_cos_i = Dot(direction_in, normal);

    if (signed_cos_o * signed_cos_i <= 0) {
      continue;
    }

    return BasicScatter(
      direction_in,
      (exponent_ + 2) / (exponent_ + 1) * std::abs(signed_cos_i)
    );
  }
}

}
}
