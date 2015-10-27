/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>

#include "base/constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Phong : public Material<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant kd_;           // diffuse reflectivity
  Radiant ks_;           // specular reflectivity
  radiant_value_type n_; // specular exponent
  radiant_value_type p_diffuse_;

public:
  Phong(Radiant const& kd, Radiant const& ks, radiant_value_type n) noexcept
    : kd_(kd), ks_(ks), n_(n), p_diffuse_(kd.sum() / (kd.sum() + ks.sum())) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::diffuse; }

  Radiant bsdf(vector3_type const& direction_i,
               vector3_type const& direction_o,
               vector3_type const& normal) const noexcept {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return Radiant();
    }
    auto const cos_alpha =
      dot(direction_o, 2 * dot(direction_i, normal) * normal - direction_i);
    return kd_ / kPI
      + ks_ * (n_ + 2) / (2 * kPI)
      * std::pow(std::max<radiant_value_type>(0, cos_alpha), n_);
  }

  scatter_type sampleScatter(Radiant const&,
                             vector3_type const& direction_i,
                             vector3_type const& normal,
                             Sampler* sampler) const {
    auto const direction_o = sampler->uniform<radiant_value_type>() < p_diffuse_
      ? sampleDirectionFromDiffuseComponent(direction_i, normal, sampler)
      : sampleDirectionFromSpecularComponent(direction_i, normal, sampler);
    auto const cos_alpha =
      dot(direction_o, 2 * dot(direction_i, normal) * normal - direction_i);
    auto const psa_probability =
      p_diffuse_ / kPI
      + (1 - p_diffuse_) * (n_ + 1) / (2 * kPI)
      * std::pow(std::max<radiant_value_type>(0, cos_alpha), n_);
    return scatter_type(direction_o,
                        bsdf(direction_i, direction_o, normal),
                        psa_probability);
  }

private:
  vector3_type
  sampleDirectionFromDiffuseComponent(vector3_type const& direction_i,
                                      vector3_type const& normal,
                                      Sampler* sampler) const {
    auto const w = dot(direction_i, normal) > 0 ? normal : -normal;
    return std::get<0>(sampler->hemispherePSA(w));
  }

  vector3_type
  sampleDirectionFromSpecularComponent(const vector3_type& direction_i,
                                       const vector3_type& normal,
                                       Sampler *sampler) const {
    for (;;) {
      auto const r0 = sampler->uniform<RealType>();
      auto const r1 = sampler->uniform<RealType>();
      auto const cos_alpha = std::pow(r0, 1 / (n_ + 1));
      auto const sin_alpha = std::sqrt(1 - std::pow(r0, 2 / (n_ + 1)));
      auto const phi = 2 * kPI * r1;

      auto const x = sin_alpha * std::cos(phi);
      auto const y = sin_alpha * std::sin(phi);
      auto const z = cos_alpha;

      vector3_type w = 2 * dot(direction_i, normal) * normal - direction_i;
      vector3_type u, v;
      std::tie(u, v) = orthonormalBasis(w);
      auto const direction_o = u * x + v * y + w * z;

      if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
        continue;
      }

      return direction_o;
    }
  }
};

}
}
