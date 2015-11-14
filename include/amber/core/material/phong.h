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
class Phong : public SymmetricBSDF<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant kd_;
  Radiant ks_;
  radiant_value_type exponent_;

public:
  Phong(
    Radiant const& kd,
    Radiant const& ks,
    radiant_value_type exponent
  ) noexcept
  : kd_(kd),
    ks_(ks),
    exponent_(exponent)
  {}

  SurfaceType Surface() const noexcept
  {
    return SurfaceType::Diffuse;
  }

  Radiant
  BSDF(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_i = Dot(direction_i, normal);
    auto const signed_cos_o = Dot(direction_o, normal);

    if (signed_cos_i * signed_cos_o <= 0) {
      return Radiant();
    }

    auto const direction_r = 2 * signed_cos_i * normal - direction_i;
    auto const cos_alpha =
      std::max<radiant_value_type>(0, Dot(direction_r, direction_o));

    return
      kd_ / kPI +
      ks_ * (exponent_ + 2) / 2 / kPI * std::pow(cos_alpha, exponent_);
  }

  radiant_value_type
  pdf(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const rho_d = kd_.Sum();
    auto const rho_s = ks_.Sum();
    auto const rho = rho_d + rho_s;

    auto const direction_s =
      2 * Dot(direction_o, normal) * normal - direction_o;
    auto const cos_alpha = std::max<RealType>(0, Dot(direction_i, direction_s));

    auto const pdf_d = 1 / kPI;
    auto const pdf_s =
      (exponent_ + 1) / 2 / kPI * std::pow(cos_alpha, exponent_) /
      std::abs(Dot(direction_i, normal));

    return (rho_d * pdf_d + rho_s * pdf_s) / rho;
  }

  scatter_type
  Sample(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    auto const rho_d = kd_.Sum();
    auto const rho_s = ks_.Sum();
    auto const rho = rho_d + rho_s;

    if (sampler->uniform(rho) < rho_d) {
      // sample from the diffuse component
      auto const direction_i = sampleDiffuse(direction_o, normal, sampler);
      return scatter_type(
        direction_i,
        kd_ / rho_d * rho
      );
    } else {
      // sample from the specular component
      auto const direction_i = sampleSpecular(direction_o, normal, sampler);
      return scatter_type(
        direction_i,
        ks_ / rho_s * rho *
        (exponent_ + 2) / (exponent_ + 1) * std::abs(Dot(direction_i, normal))
      );
    }
  }

private:
  vector3_type
  sampleDiffuse(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    auto const w = Dot(direction_o, normal) > 0 ? normal : -normal;
    return std::get<0>(sampler->hemispherePSA(w));
  }

  vector3_type
  sampleSpecular(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    for (;;) {
      auto const cos_alpha =
        std::pow(sampler->uniform<RealType>(), 1 / (exponent_ + 1));
      auto const sin_alpha = std::sqrt(1 - cos_alpha * cos_alpha);
      auto const phi = 2 * kPI * sampler->uniform<RealType>();

      auto const w = 2 * Dot(direction_o, normal) * normal - direction_o;
      vector3_type u, v;
      std::tie(u, v) = OrthonormalBasis(w);

      auto const direction_i =
        sin_alpha * std::cos(phi) * u +
        sin_alpha * std::sin(phi) * v +
        cos_alpha * w;

      if (Dot(direction_i, normal) * Dot(direction_o, normal) > 0) {
        return direction_i;
      }
    }
  }
};

}
}
}
