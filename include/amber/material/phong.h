/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "constant.h"
#include "material/symmetric_bsdf.h"

namespace amber {
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

  SurfaceType surfaceType() const noexcept { return SurfaceType::Diffuse; }
  Radiant emittance() const noexcept { return Radiant(); }

  Radiant
  bsdf(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_i = dot(direction_i, normal);
    auto const signed_cos_o = dot(direction_o, normal);

    if (signed_cos_i * signed_cos_o <= 0) {
      return Radiant();
    }

    auto const direction_r = 2 * signed_cos_i * normal - direction_i;
    auto const cos_alpha =
      std::max<radiant_value_type>(0, dot(direction_r, direction_o));

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
    auto const rho_d = kd_.sum();
    auto const rho_s = ks_.sum();
    auto const rho = rho_d + rho_s;

    auto const direction_s =
      2 * dot(direction_o, normal) * normal - direction_o;
    auto const cos_alpha = std::max<RealType>(0, dot(direction_i, direction_s));

    auto const pdf_d = 1 / kPI;
    auto const pdf_s =
      (exponent_ + 1) / 2 / kPI * std::pow(cos_alpha, exponent_) /
      std::abs(dot(direction_i, normal));

    return (rho_d * pdf_d + rho_s * pdf_s) / rho;
  }

  scatter_type
  sample(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    auto const rho_d = kd_.sum();
    auto const rho_s = ks_.sum();
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
        (exponent_ + 2) / (exponent_ + 1) * std::abs(dot(direction_i, normal))
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
    auto const w = dot(direction_o, normal) > 0 ? normal : -normal;
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

      auto const w = 2 * dot(direction_o, normal) * normal - direction_o;
      vector3_type u, v;
      std::tie(u, v) = orthonormalBasis(w);

      auto const direction_i =
        sin_alpha * std::cos(phi) * u +
        sin_alpha * std::sin(phi) * v +
        cos_alpha * w;

      if (dot(direction_i, normal) * dot(direction_o, normal) > 0) {
        return direction_i;
      }
    }
  }
};

}
}
