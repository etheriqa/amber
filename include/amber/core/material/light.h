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
class Light : public SymmetricBSDF<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant radiance_;

public:
  explicit Light(Radiant const& radiance) noexcept : radiance_(radiance) {}

  SurfaceType Surface() const noexcept
  {
    return SurfaceType::Light;
  }

  Radiant Radiance() const noexcept { return radiance_; }

  Radiant
  BSDF(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept
  {
    return Radiant();
  }

  radiant_value_type
  pdf(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept
  {
    return 1 / kPI;
  }

  scatter_type
  Sample(
    vector3_type const& direction_i,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    auto const w = Dot(direction_i, normal) > 0 ? normal : -normal;
    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = sampler->hemispherePSA(w);
    return scatter_type(direction_o, Radiant(0));
  }
};

}
}
}
