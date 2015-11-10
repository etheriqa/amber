/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/constant.h"
#include "material/symmetric_bsdf.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Light : public SymmetricBSDF<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant radiance_;

public:
  explicit Light(Radiant const& radiance) noexcept : radiance_(radiance) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::Light; }
  Radiant emittance() const noexcept { return radiance_; }

  Radiant
  bsdf(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept
  {
    return Radiant();
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
  sample(
    vector3_type const& direction_i,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    auto const w = dot(direction_i, normal) > 0 ? normal : -normal;
    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = sampler->hemispherePSA(w);
    return scatter_type(direction_o, Radiant(0));
  }
};

}
}
