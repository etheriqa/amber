/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>

#include "base/constant.h"
#include "material/symmetric_bsdf.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Specular : public SymmetricBSDF<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant ks_;

public:
  explicit Specular(Radiant const& ks) noexcept : ks_(ks) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }
  bool isEmissive() const noexcept { return false; }
  Radiant emittance() const noexcept { return Radiant(); }

  Radiant
  bsdf(vector3_type const& direction_i,
       vector3_type const& direction_o,
       vector3_type const& normal) const noexcept {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return Radiant();
    } else {
      return ks_ * kDiracDelta;
    }
  }

  radiant_value_type
  scatterPDF(vector3_type const&,
             vector3_type const&,
             vector3_type const&) const noexcept {
    return kDiracDelta;
  }

  scatter_type
  sampleScatter(vector3_type const& direction_i,
                vector3_type const& normal,
                Sampler*) const {
    return specularScatters(direction_i, normal).front();
  }

  std::vector<scatter_type>
  specularScatters(vector3_type const& direction_i,
                   vector3_type const& normal) const {
    return {
      scatter_type(2 * dot(direction_i, normal) * normal - direction_i,
                   ks_ * kDiracDelta,
                   kDiracDelta),
    };
  }
};

}
}
