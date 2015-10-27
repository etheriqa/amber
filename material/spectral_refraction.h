/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "material/refraction.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class SpectralRefraction : public Material<Radiant, RealType> {
public:
  using scatter_type    = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type    = typename Material<Radiant, RealType>::vector3_type;

private:
  using refraction_type = Refraction<Radiant, RealType>;

  Radiant refractive_indices_;

public:
  explicit SpectralRefraction(Radiant const& refractive_indices) noexcept
    : refractive_indices_(refractive_indices) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }

  std::vector<scatter_type>
  specularScatters(Radiant const& radiant,
                   vector3_type const& direction_i,
                   vector3_type const& normal) const {
    auto const r = refraction_type(refractive_indices_.r());
    auto const g = refraction_type(refractive_indices_.g());
    auto const b = refraction_type(refractive_indices_.b());

    std::vector<scatter_type> scatters;
    for (auto scatter : r.specularScatters(Radiant(), direction_i, normal)) {
      auto const filter = Radiant(1, 0, 0);
      scatter.bsdf *= filter;
      scatter.psa_probability *= (radiant * filter).sum() / radiant.sum();
      if (scatter.psa_probability > 0) {
        scatters.push_back(scatter);
      }
    }
    for (auto scatter : g.specularScatters(Radiant(), direction_i, normal)) {
      auto const filter = Radiant(0, 1, 0);
      scatter.bsdf *= filter;
      scatter.psa_probability *= (radiant * filter).sum() / radiant.sum();
      if (scatter.psa_probability > 0) {
        scatters.push_back(scatter);
      }
    }
    for (auto scatter : b.specularScatters(Radiant(), direction_i, normal)) {
      auto const filter = Radiant(0, 0, 1);
      scatter.bsdf *= filter;
      scatter.psa_probability *= (radiant * filter).sum() / radiant.sum();
      if (scatter.psa_probability > 0) {
        scatters.push_back(scatter);
      }
    }
    return scatters;
  }
};

}
}
