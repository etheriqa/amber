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
private:
  using material_type          = Material<Radiant, RealType>;
  using refraction_type        = Refraction<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

  radiant_type refractive_indices_;

public:
  explicit SpectralRefraction(const radiant_type& refractive_indices) noexcept
    : refractive_indices_(refractive_indices) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }

  std::vector<scattering_sample_type>
  scatteringCandidates(const radiant_type& radiant,
                       const vector3_type& direction_i,
                       const vector3_type& normal) const {
    const auto r = refraction_type(refractive_indices_.r());
    const auto g = refraction_type(refractive_indices_.g());
    const auto b = refraction_type(refractive_indices_.b());

    std::vector<scattering_sample_type> candidates;
    for (auto s : r.scatteringCandidates(radiant_type(), direction_i, normal)) {
      const auto filter = radiant_type(1, 0, 0);
      s.bsdf *= filter;
      s.psa_probability *= (radiant * filter).sum() / radiant.sum();
      if (s.psa_probability > 0) {
        candidates.push_back(s);
      }
    }
    for (auto s : g.scatteringCandidates(radiant_type(), direction_i, normal)) {
      const auto filter = radiant_type(0, 1, 0);
      s.bsdf *= filter;
      s.psa_probability *= (radiant * filter).sum() / radiant.sum();
      if (s.psa_probability > 0) {
        candidates.push_back(s);
      }
    }
    for (auto s : b.scatteringCandidates(radiant_type(), direction_i, normal)) {
      const auto filter = radiant_type(0, 0, 1);
      s.bsdf *= filter;
      s.psa_probability *= (radiant * filter).sum() / radiant.sum();
      if (s.psa_probability > 0) {
        candidates.push_back(s);
      }
    }
    return candidates;
  }
};

}
}
