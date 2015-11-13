/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>

#include "constant.h"
#include "symmetric_bsdf.h"

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

  SurfaceType Surface() const noexcept
  {
    return amber::SurfaceType::Specular;
  }

  Radiant
  BSDF(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    auto const signed_cos_i = Dot(direction_i, normal);
    auto const signed_cos_o = Dot(direction_o, normal);

    if (signed_cos_i * signed_cos_o <= 0) {
      return Radiant();
    } else {
      return ks_ * kDiracDelta / std::abs(signed_cos_i);
    }
  }

  radiant_value_type
  pdf(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept
  {
    return kDiracDelta;
  }

  std::vector<scatter_type>
  distribution(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    return {
      scatter_type(2 * Dot(direction_o, normal) * normal - direction_o, ks_),
    };
  }
};

}
}
