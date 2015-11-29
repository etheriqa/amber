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

#include <algorithm>
#include <cmath>
#include <numeric>

#include "core/constant.h"
#include "core/geometry.h"
#include "core/material.h"

namespace {

std::double_t const kReflectance = 1;
std::double_t const kTransmittance = 1;

}

namespace amber {
namespace core {
namespace material {

template <typename Radiant, typename RealType>
class Refraction : public Material<Radiant, RealType>
{
private:
  using typename Material<Radiant, RealType>::radiant_value_type;
  using typename Material<Radiant, RealType>::scatter_type;
  using typename Material<Radiant, RealType>::vector3_type;

  radiant_value_type ior_, r0_;

public:
  explicit Refraction(radiant_value_type ior) noexcept
    : ior_(ior), r0_(Fresnel(ior)) {}

  SurfaceType Surface() const noexcept
  {
    return SurfaceType::Specular;
  }

  Radiant
  BSDF(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_alpha = Dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // total reflection
      return Radiant(kReflectance * kDiracDelta);
    }

    // transmittance is needed to scale by relative IOR
    auto const rho_r = Schlick(r0_, std::abs(signed_cos_alpha));
    auto const rho_t = (1 - rho_r) / (ior * ior);
    auto const rho = rho_r + rho_t;

    auto const signed_cos_i = Dot(direction_i, normal);

    if (signed_cos_alpha * signed_cos_i > 0) {
      // partial reflection
      return Radiant(rho_r / rho * kReflectance * kDiracDelta);
    } else {
      // refraction
      return Radiant(rho_t / rho * kTransmittance * kDiracDelta);
    }
  }

  Radiant
  AdjointBSDF(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_alpha = Dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // total reflection
      return Radiant(kReflectance * kDiracDelta);
    }

    // no scaling is required unlike light transport
    auto const rho_r = Schlick(r0_, std::abs(signed_cos_alpha));
    auto const rho_t = 1 - rho_r;

    auto const signed_cos_i = Dot(direction_i, normal);

    if (signed_cos_alpha * signed_cos_i > 0) {
      // partial reflection
      return Radiant(rho_r * kReflectance * kDiracDelta);
    } else {
      // refraction
      return Radiant(rho_t * kTransmittance * kDiracDelta);
    }
  }

  radiant_value_type
  PDFLight(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_alpha = Dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // total reflection
      return kDiracDelta;
    }

    // transmittance is needed to scale by relative IOR
    auto const rho_r = Schlick(r0_, std::abs(signed_cos_alpha));
    auto const rho_t = (1 - rho_r) / (ior * ior);
    auto const rho = rho_r + rho_t;

    auto const signed_cos_i = Dot(direction_i, normal);

    if (signed_cos_alpha * signed_cos_i > 0) {
      // partial reflection
      return rho_r / rho * kDiracDelta;
    } else {
      // refraction
      return rho_t / rho * kDiracDelta;
    }
  }

  radiant_value_type
  PDFImportance(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_alpha = Dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

    if (squared_cos_beta < 0) {
      // total reflection
      return kDiracDelta;
    }

    // no scaling is required unlike light transport
    auto const rho_r = Schlick(r0_, std::abs(signed_cos_alpha));
    auto const rho_t = 1 - rho_r;

    auto const signed_cos_i = Dot(direction_i, normal);

    if (signed_cos_alpha * signed_cos_i > 0) {
      // partial reflection
      return rho_r * kDiracDelta;
    } else {
      // refraction
      return rho_t * kDiracDelta;
    }
  }

  std::vector<scatter_type>
  DistributionLight(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    auto const signed_cos_alpha = Dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);
    auto const direction_r =
      PerfectReflection(direction_o, normal, signed_cos_alpha);

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
    auto const rho_r = Schlick(r0_, cos_alpha);
    auto const rho_t = (1 - rho_r) / (ior * ior);

    return {
      // partial reflection
      scatter_type(direction_r, Radiant(rho_r * kReflectance)),
      // refraction
      scatter_type(direction_t, Radiant(rho_t * kTransmittance)),
    };
  }

  std::vector<scatter_type>
  DistributionImportance(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    auto const signed_cos_alpha = Dot(direction_o, normal);
    auto const ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
    auto const squared_cos_beta =
      1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);
    auto const direction_r =
      PerfectReflection(direction_o, normal, signed_cos_alpha);

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
    auto const rho_r = Schlick(r0_, cos_alpha);
    auto const rho_t = 1 - rho_r;

    return {
      // partial reflection
      scatter_type(direction_r, Radiant(rho_r * kReflectance)),
      // refraction
      scatter_type(direction_t, Radiant(rho_t * kTransmittance)),
    };
  }

private:
  static RealType Fresnel(RealType ior) noexcept {
    return std::pow((ior - 1) / (ior + 1), 2);
  }

  static RealType Schlick(RealType r0, RealType cos_theta) noexcept {
    return r0 + (1 - r0) * std::pow(1 - cos_theta, 5);
  }
};

}
}
}
