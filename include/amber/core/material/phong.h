// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include "core/constant.h"
#include "core/geometry.h"
#include "core/symmetric_bsdf.h"

namespace amber {
namespace core {
namespace material {

template <typename Radiant, typename RealType>
class Phong : public SymmetricBSDF<Radiant, RealType>
{
private:
  using typename Material<Radiant, RealType>::radiant_value_type;
  using typename Material<Radiant, RealType>::scatter_type;
  using typename Material<Radiant, RealType>::unit_vector3_type;

  Radiant ks_;
  RealType exponent_;

public:
  Phong(
    Radiant const& ks,
    RealType const exponent
  ) noexcept
  : ks_(ks)
  , exponent_(exponent)
  {}

  SurfaceType Surface() const noexcept
  {
    return SurfaceType::Diffuse;
  }

  Radiant const
  BSDF(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_i = Dot(direction_i, normal);
    auto const signed_cos_o = Dot(direction_o, normal);

    if (signed_cos_i * signed_cos_o <= 0) {
      return Radiant();
    }

    auto const direction_s =
      PerfectReflection(direction_i, normal, signed_cos_i);
    auto const cos_alpha =
      std::max<radiant_value_type>(0, Dot(direction_s, direction_o));

    return ks_ * (exponent_ + 2) / 2 / kPI * std::pow(cos_alpha, exponent_);
  }

  radiant_value_type const
  PDF(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept
  {
    auto const direction_s = PerfectReflection(direction_o, normal);
    auto const cos_alpha = std::max<RealType>(0, Dot(direction_i, direction_s));

    return
      (exponent_ + 1) / (2 * kPI) *
      std::pow(cos_alpha, exponent_) /
      std::abs(Dot(direction_i, normal));
  }

  scatter_type
  Sample(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal,
    Sampler& sampler
  ) const
  {
    auto const direction_r = PerfectReflection(direction_o, normal);

    for (;;) {
      auto const direction_i =
        std::get<0>(CosinePower(direction_r, exponent_, sampler));
      if (Dot(direction_i, normal) * Dot(direction_o, normal) > 0) {
        return scatter_type(
          direction_i,
          ks_ *
            (exponent_ + 2) / (exponent_ + 1) *
            std::abs(Dot(direction_i, normal))
        );
      }
    }
  }
};

}
}
}
