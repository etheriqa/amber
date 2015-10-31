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
class Specular : public Material<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant ks_;

public:
  explicit Specular(Radiant const& ks) noexcept : ks_(ks) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }

  Radiant bsdf(vector3_type const& direction_i,
               vector3_type const& direction_o,
               vector3_type const& normal) const noexcept {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return Radiant();
    } else {
      return ks_ * kDiracDelta;
    }
  }

  radiant_value_type pdf(vector3_type const&,
                         vector3_type const&,
                         vector3_type const&) const noexcept {
    return kDiracDelta;
  }

  std::vector<scatter_type>
  specularScatters(vector3_type const& direction_i,
                   vector3_type const& normal) const {
    return std::vector<scatter_type>({
      scatter_type(2 * dot(direction_i, normal) * normal - direction_i,
                   ks_ * kDiracDelta,
                   kDiracDelta)});
  }
};

}
}