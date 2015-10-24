/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Refraction : public Material<Radiant, RealType> {
private:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

  real_type refractive_index_;

public:
  explicit Refraction(real_type refractive_index) noexcept
    : refractive_index_(refractive_index) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }

  std::vector<scattering_sample_type>
  scatteringCandidates(const radiant_type&,
                       const vector3_type& direction_i,
                       const vector3_type& normal) const {
    std::vector<scattering_sample_type> candidates;
    scattering_sample_type sample;

    const auto signed_cos_i = dot(direction_i, normal);
    const auto reflection_direction_o = 2 * signed_cos_i * normal - direction_i;
    const auto ri = signed_cos_i > 0
      ? 1 / refractive_index_
      : refractive_index_;
    const auto squared_cos_o =
      1 - (1 - signed_cos_i * signed_cos_i) * (ri * ri);

    if (squared_cos_o < 0) {
      // Full reflection
      sample.direction_o = reflection_direction_o;
      sample.bsdf = radiant_type(1 / kEPS);
      sample.psa_probability = 1 / kEPS;
      candidates.push_back(sample);
      return candidates;
    }

    const auto cos_i = std::abs(signed_cos_i);
    const auto cos_o = std::sqrt(squared_cos_o);
    const auto p_reflection = fresnel(std::acos(cos_i), std::acos(cos_o));

    // Partial reflection
    sample.direction_o = reflection_direction_o;
    sample.bsdf = radiant_type(p_reflection / kEPS);
    sample.psa_probability = p_reflection / kEPS;
    candidates.push_back(sample);

    // Refraction
    sample.direction_o = (cos_i - cos_o / ri) * normal - direction_i;
    sample.bsdf = radiant_type((1 - p_reflection) / kEPS);
    sample.psa_probability = (1 - p_reflection) / kEPS;
    candidates.push_back(sample);

    return candidates;
  }

private:
  static real_type fresnel(real_type alpha, real_type beta) noexcept
  {
    const auto s = std::sin(alpha - beta) / std::sin(alpha + beta);
    const auto t = std::tan(alpha - beta) / std::tan(alpha + beta);
    return (s * s + t * t) / 2;
  }
};

}
}
