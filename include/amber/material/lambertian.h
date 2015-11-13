/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "constant.h"
#include "symmetric_bsdf.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Lambertian : public SymmetricBSDF<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant kd_;

public:
  explicit Lambertian(Radiant const& kd) noexcept : kd_(kd) {}

  SurfaceType Surface() const noexcept
  {
    return amber::SurfaceType::Diffuse;
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
      return kd_ / kPI;
    }
  }

  radiant_value_type
  pdf(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept
  {
    return 1 / kPI;
  }

  scatter_type
  Sample(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    auto const w = Dot(direction_o, normal) > 0 ? normal : -normal;
    vector3_type direction_i;
    std::tie(direction_i, std::ignore) = sampler->hemispherePSA(w);
    return scatter_type(direction_i, kd_);
  }
};

}
}
