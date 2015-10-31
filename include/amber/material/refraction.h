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

  Radiant bsdf(vector3_type const& direction_i,
               vector3_type const& direction_o,
               vector3_type const& normal) const noexcept {
    auto const signed_cos_alpha =
      dot(direction_i, normal);
    auto const ri = signed_cos_alpha > 0
      ? 1 / refractive_index_
      : refractive_index_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ri * ri);

    if (squared_cos_beta < 0) {
      // Full reflection
      return Radiant(kDiracDelta);
    }

    auto const cos_alpha = std::abs(signed_cos_alpha);
    auto const cos_beta = std::sqrt(squared_cos_beta);
    auto const p_reflection =
      fresnel(std::acos(cos_alpha), std::acos(cos_beta));

    if (!std::isfinite(p_reflection)) { // XXX
      return Radiant(kDiracDelta);
    }

    auto const signed_cos_o = dot(direction_o, normal);
    if (signed_cos_alpha * signed_cos_o > 0) {
      // Partial reflection
      return Radiant(p_reflection * kDiracDelta);
    } else {
      // Refraction
      return Radiant((1 - p_reflection) * kDiracDelta);
    }
  }

  radiant_value_type pdf(vector3_type const& direction_i,
                         vector3_type const& direction_o,
                         vector3_type const& normal) const noexcept {
    auto const signed_cos_alpha =
      dot(direction_i, normal);
    auto const ri = signed_cos_alpha > 0
      ? 1 / refractive_index_
      : refractive_index_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ri * ri);

    if (squared_cos_beta < 0) {
      // Full reflection
      return kDiracDelta;
    }

    auto const cos_alpha = std::abs(signed_cos_alpha);
    auto const cos_beta = std::sqrt(squared_cos_beta);
    auto const p_reflection =
      fresnel(std::acos(cos_alpha), std::acos(cos_beta));

    if (!std::isfinite(p_reflection)) { // XXX
      return kDiracDelta;
    }

    auto const signed_cos_o = dot(direction_o, normal);
    if (signed_cos_alpha * signed_cos_o > 0) {
      // Partial reflection
      return p_reflection * kDiracDelta;
    } else {
      // Refraction
      return (1 - p_reflection) * kDiracDelta;
    }
  }

  std::vector<scatter_type>
  specularScatters(vector3_type const& direction_i,
                   vector3_type const& normal) const {
    auto const signed_cos_alpha =
      dot(direction_i, normal);
    auto const reflection_direction =
      2 * signed_cos_alpha * normal - direction_i;
    auto const ri = signed_cos_alpha > 0
      ? 1 / refractive_index_
      : refractive_index_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ri * ri);

    if (squared_cos_beta < 0) {
      return std::vector<scatter_type>({
        // Full reflection
        scatter_type(reflection_direction,
                     Radiant(kDiracDelta),
                     kDiracDelta)});
    }

    auto const cos_alpha = std::abs(signed_cos_alpha);
    auto const cos_beta = std::sqrt(squared_cos_beta);
    auto const p_reflection =
      fresnel(std::acos(cos_alpha), std::acos(cos_beta));

    return std::vector<scatter_type>({
      // Partial reflection
      scatter_type(reflection_direction,
                   Radiant(p_reflection * kDiracDelta),
                   p_reflection * kDiracDelta),
      // Refraction
      scatter_type((cos_alpha - cos_beta / ri) * normal - direction_i,
                   Radiant((1 - p_reflection) * kDiracDelta),
                   (1 - p_reflection) * kDiracDelta)});
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
