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
#include "core/symmetric_bsdf.h"

namespace amber {
namespace core {
namespace material {

template <typename Radiant, typename RealType>
class Light : public SymmetricBSDF<Radiant, RealType>
{
private:
  using typename Material<Radiant, RealType>::radiant_value_type;
  using typename Material<Radiant, RealType>::scatter_type;
  using typename Material<Radiant, RealType>::unit_vector3_type;

  Radiant radiance_;

public:
  explicit Light(Radiant const& radiance) noexcept : radiance_(radiance) {}

  SurfaceType Surface() const noexcept
  {
    return SurfaceType::Light;
  }

  Radiant const Irradiance() const noexcept { return radiance_ * kPI; }

  Radiant const
  Radiance(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept
  {
    if (Dot(direction_o, normal) <= 0) {
      return Radiant();
    } else {
      return radiance_;
    }
  }

  Radiant const
  BSDF(
    unit_vector3_type const&,
    unit_vector3_type const&,
    unit_vector3_type const&
  ) const noexcept
  {
    return Radiant();
  }

  radiant_value_type const
  PDF(
    unit_vector3_type const&,
    unit_vector3_type const&,
    unit_vector3_type const&
  ) const noexcept
  {
    return 1 / kPI;
  }

  scatter_type
  Sample(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal,
    Sampler& sampler
  ) const
  {
    auto const w = Dot(direction_o, normal) > 0 ? normal : -normal;
    unit_vector3_type direction_i;
    std::tie(direction_i, std::ignore) = HemispherePSA(w, sampler);
    return scatter_type(direction_i, Radiant());
  }
};

}
}
}
