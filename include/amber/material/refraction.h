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

#include "constant.h"
#include "material/material.h"

namespace {

double const kReflectance = 0.95;
double const kTransmittance = 0.95;

}

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

  SurfaceType surfaceType() const noexcept { return SurfaceType::Specular; }
  Radiant emittance() const noexcept { return Radiant(); }

  Radiant
  bsdf(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_alpha = dot(direction_i, normal);
    auto const cos_alpha = std::abs(signed_cos_alpha);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // total reflection
      return Radiant(kReflectance * kDiracDelta / cos_alpha);
    }

    auto const signed_cos_o = dot(direction_o, normal);
    auto const rho_r = schlick(r0_, std::abs(signed_cos_alpha));
    auto const rho_t = 1 - rho_r;

    if (signed_cos_alpha * signed_cos_o > 0) {
      // partial reflection
      return Radiant(rho_r * kReflectance * kDiracDelta / cos_alpha);
    } else {
      // refraction
      return Radiant(rho_t * kTransmittance * kDiracDelta / cos_alpha);
    }
  }

  radiant_value_type
  pdfLight(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_alpha = dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // total reflection
      return kDiracDelta;
    }

    // transmittance is needed to scale by relative IOR
    auto const rho_r = schlick(r0_, std::abs(signed_cos_alpha));
    auto const rho_t = (1 - rho_r) / (ior * ior);
    auto const rho = rho_r + rho_t;

    auto const signed_cos_i = dot(direction_i, normal);

    if (signed_cos_alpha * signed_cos_i > 0) {
      // partial reflection
      return rho_r / rho * kDiracDelta;
    } else {
      // refraction
      return rho_t / rho * kDiracDelta;
    }
  }

  radiant_value_type
  pdfImportance(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_alpha = dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // total reflection
      return kDiracDelta;
    }

    // no scaling is required unlike light transport
    auto const rho_r = schlick(r0_, std::abs(signed_cos_alpha));
    auto const rho_t = 1 - rho_r;

    auto const signed_cos_i = dot(direction_i, normal);

    if (signed_cos_alpha * signed_cos_i > 0) {
      // partial reflection
      return rho_r * kDiracDelta;
    } else {
      // refraction
      return rho_t * kDiracDelta;
    }
  }

  std::vector<scatter_type>
  distributionLight(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    auto const signed_cos_alpha = dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);
    auto const direction_r =
      2 * signed_cos_alpha * normal - direction_o;

    if (squared_cos_beta < 0) {
      return {
        // total reflection
        scatter_type(direction_r, Radiant(kReflectance)),
      };
    }

    auto const cos_alpha = std::abs(signed_cos_alpha);
    auto const cos_beta = std::sqrt(squared_cos_beta);
    auto const direction_t =
      -ior * direction_o +
      ((signed_cos_alpha < 0 ? 1 : -1) * cos_beta + ior * signed_cos_alpha) *
      normal;

    // transmittance is needed to scale by relative IOR
    auto const rho_r = schlick(r0_, cos_alpha);
    auto const rho_t = (1 - rho_r) / (ior * ior);

    return {
      // partial reflection
      scatter_type(direction_r, Radiant(rho_r * kReflectance)),
      // refraction
      scatter_type(direction_t, Radiant(rho_t * kTransmittance)),
    };
  }

  std::vector<scatter_type>
  distributionImportance(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    auto const signed_cos_alpha = dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);
    auto const direction_r =
      2 * signed_cos_alpha * normal - direction_o;

    if (squared_cos_beta < 0) {
      return {
        // total reflection
        scatter_type(direction_r, Radiant(kReflectance)),
      };
    }

    auto const cos_alpha = std::abs(signed_cos_alpha);
    auto const cos_beta = std::sqrt(squared_cos_beta);
    auto const direction_t =
      -ior * direction_o +
      ((signed_cos_alpha < 0 ? 1 : -1) * cos_beta + ior * signed_cos_alpha) *
      normal;

    // no scaling is required unlike light transport
    auto const rho_r = schlick(r0_, cos_alpha);
    auto const rho_t = 1 - rho_r;

    return {
      // partial reflection
      scatter_type(direction_r, Radiant(rho_r * kReflectance)),
      // refraction
      scatter_type(direction_t, Radiant(rho_t * kTransmittance)),
    };
  }

private:
  static RealType fresnel(RealType ior) noexcept {
    return std::pow((ior - 1) / (ior + 1), 2);
  }

  static RealType schlick(RealType r0, RealType cos_theta) noexcept {
    return r0 + (1 - r0) * std::pow(1 - cos_theta, 5);
  }
};

}
}
