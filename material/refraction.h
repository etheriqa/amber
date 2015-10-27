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
class Refraction : public Material<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  radiant_value_type refractive_index_;

public:
  explicit Refraction(radiant_value_type refractive_index) noexcept
    : refractive_index_(refractive_index) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }

  std::vector<scatter_type>
  specularScatters(Radiant const&,
                   vector3_type const& direction_i,
                   vector3_type const& normal) const {
    auto const signed_cos_i =
      dot(direction_i, normal);
    auto const reflection_direction_o =
      2 * signed_cos_i * normal - direction_i;
    auto const ri = signed_cos_i > 0
      ? 1 / refractive_index_
      : refractive_index_;
    auto const squared_cos_o =
      1 - (1 - signed_cos_i * signed_cos_i) * (ri * ri);

    if (squared_cos_o < 0) {
      return std::vector<scatter_type>({
        // Full reflection
        scatter_type(reflection_direction_o,
                     Radiant(1 / kEPS),
                     1 / kEPS)});
    }

    auto const cos_i = std::abs(signed_cos_i);
    auto const cos_o = std::sqrt(squared_cos_o);
    auto const p_reflection = fresnel(std::acos(cos_i), std::acos(cos_o));

    return std::vector<scatter_type>({
      // Partial reflection
      scatter_type(reflection_direction_o,
                   Radiant(p_reflection / kEPS),
                   p_reflection / kEPS),
      // Refraction
      scatter_type((cos_i - cos_o / ri) * normal - direction_i,
                   Radiant((1 - p_reflection) / kEPS),
                   (1 - p_reflection) / kEPS)});
  }

private:
  RealType static fresnel(RealType alpha, RealType beta) noexcept
  {
    auto const s = std::sin(alpha - beta) / std::sin(alpha + beta);
    auto const t = std::tan(alpha - beta) / std::tan(alpha + beta);
    return (s * s + t * t) / 2;
  }
};

}
}
