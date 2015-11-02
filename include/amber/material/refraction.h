/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>

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
  radiant_value_type ior_, r0_;

public:
  explicit Refraction(radiant_value_type ior) noexcept
    : ior_(ior), r0_(fresnel(ior)) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }
  bool isEmissive() const noexcept { return false; }
  Radiant emittance() const noexcept { return Radiant(); }

  Radiant
  bsdf(vector3_type const& direction_i,
       vector3_type const& direction_o,
       vector3_type const& normal) const noexcept {
    auto const signed_cos_alpha = dot(direction_i, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // Full reflection
      return Radiant(kDiracDelta);
    }

    auto const signed_cos_o = dot(direction_o, normal);
    auto const p_reflection = schlick(r0_, std::abs(signed_cos_alpha));
    auto const p_refraction = 1 - p_reflection;

    if (signed_cos_alpha * signed_cos_o > 0) {
      // Partial reflection
      return Radiant(p_reflection * kDiracDelta);
    } else {
      // Refraction
      return Radiant(p_refraction * kDiracDelta);
    }
  }

  radiant_value_type
  lightScatterPDF(vector3_type const& direction_i,
                  vector3_type const& direction_o,
                  vector3_type const& normal) const noexcept {
    auto const signed_cos_alpha = dot(direction_i, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // Full reflection
      return kDiracDelta;
    }

    auto const signed_cos_o = dot(direction_o, normal);
    auto const p_reflection = schlick(r0_, std::abs(signed_cos_alpha));
    auto const p_refraction = 1 - p_reflection;

    if (signed_cos_alpha * signed_cos_o > 0) {
      // Partial reflection
      return p_reflection * kDiracDelta;
    } else {
      // Refraction
      return p_refraction * kDiracDelta;
    }
  }

  radiant_value_type
  importanceScatterPDF(vector3_type const& direction_i,
                       vector3_type const& direction_o,
                       vector3_type const& normal) const noexcept {
    auto const signed_cos_alpha = dot(direction_i, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // Full reflection
      return kDiracDelta;
    }

    auto const signed_cos_o = dot(direction_o, normal);
    auto const p_reflection = schlick(r0_, std::abs(signed_cos_alpha));
    auto const p_refraction = (1 - p_reflection) * ior * ior;
    auto const p_sum = p_reflection + p_refraction;

    if (signed_cos_alpha * signed_cos_o > 0) {
      // Partial reflection
      return p_reflection / p_sum * kDiracDelta;
    } else {
      // Refraction
      return p_refraction / p_sum * kDiracDelta;
    }
  }

  scatter_type
  sampleLightScatter(vector3_type const& direction_i,
                     vector3_type const& normal,
                     Sampler* sampler) const {
    auto const scatters = specularLightScatters(direction_i, normal);
    auto const p = sampler->uniform(
      std::accumulate(scatters.begin(), scatters.end(), radiant_value_type(),
        [](auto const& acc, auto const& scatter){
          return acc + scatter.psa_probability;
        }));
    radiant_value_type sum_p = 0;
    return *std::find_if(scatters.begin(), scatters.end(),
      [&](auto const& scatter){
        sum_p += scatter.psa_probability;
        return p < sum_p;
      });
  }

  scatter_type
  sampleImportanceScatter(vector3_type const& direction_i,
                          vector3_type const& normal,
                          Sampler* sampler) const {

    auto const scatters = specularImportanceScatters(direction_i, normal);
    auto const p = sampler->uniform(
      std::accumulate(scatters.begin(), scatters.end(), radiant_value_type(),
        [](auto const& acc, auto const& scatter){
          return acc + scatter.psa_probability;
        }));
    radiant_value_type sum_p = 0;
    return *std::find_if(scatters.begin(), scatters.end(),
      [&](auto const& scatter){
        sum_p += scatter.psa_probability;
        return p < sum_p;
      });
  }

  std::vector<scatter_type>
  specularLightScatters(vector3_type const& direction_i,
                        vector3_type const& normal) const {
    auto const signed_cos_alpha = dot(direction_i, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);
    auto const reflection_direction =
      2 * signed_cos_alpha * normal - direction_i;

    if (squared_cos_beta < 0) {
      return {
        // Full reflection
        scatter_type(reflection_direction,
                     Radiant(kDiracDelta),
                     kDiracDelta),
      };
    }

    auto const cos_alpha = std::abs(signed_cos_alpha);
    auto const cos_beta = std::sqrt(squared_cos_beta);
    auto const refraction_direction =
      -ior * direction_i +
      ((signed_cos_alpha < 0 ? 1 : -1) * cos_beta + ior * signed_cos_alpha) *
      normal;
    auto const p_reflection = schlick(r0_, cos_alpha);
    auto const p_refraction = 1 - p_reflection;

    return {
      // Partial reflection
      scatter_type(reflection_direction,
                   Radiant(p_reflection * kDiracDelta),
                   p_reflection * kDiracDelta),
      // Refraction
      scatter_type(refraction_direction,
                   Radiant(p_refraction * kDiracDelta),
                   p_refraction * kDiracDelta),
    };
  }

  std::vector<scatter_type>
  specularImportanceScatters(vector3_type const& direction_i,
                             vector3_type const& normal) const {
    auto const signed_cos_alpha = dot(direction_i, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);
    auto const reflection_direction =
      2 * signed_cos_alpha * normal - direction_i;

    if (squared_cos_beta < 0) {
      return {
        // Full reflection
        scatter_type(reflection_direction,
                     Radiant(kDiracDelta),
                     kDiracDelta),
      };
    }

    auto const cos_alpha = std::abs(signed_cos_alpha);
    auto const cos_beta = std::sqrt(squared_cos_beta);
    auto const refraction_direction =
      -ior * direction_i +
      ((signed_cos_alpha < 0 ? 1 : -1) * cos_beta + ior * signed_cos_alpha) *
      normal;
    auto const p_reflection = schlick(r0_, cos_alpha);
    auto const p_refraction = (1 - p_reflection) * ior * ior;
    auto const p_sum = p_reflection + p_refraction;

    return {
      // Partial reflection
      scatter_type(reflection_direction,
                   Radiant(p_reflection * kDiracDelta),
                   p_reflection / p_sum * kDiracDelta),
      // Refraction
      scatter_type(refraction_direction,
                   Radiant(p_refraction * kDiracDelta),
                   p_refraction / p_sum * kDiracDelta),
    };
  }

private:
  static RealType fresnel(RealType ior) noexcept {
    return std::pow((1 - ior) / (1 + ior), 2);
  }

  static RealType schlick(RealType r0, RealType cos_theta) noexcept {
    return r0 + (1 - r0) * std::pow(1 - cos_theta, 5);
  }
};

}
}
