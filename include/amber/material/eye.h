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
class Eye : public SymmetricBSDF<Radiant, RealType>
{
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

  Eye() noexcept {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::Eye; }
  Radiant emittance() const noexcept { return Radiant(); }

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
    return kDiracDelta;
  }

  std::vector<scatter_type>
  distribution(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    return {
      scatter_type(direction_o, Radiant(1)),
    }
  }
};

}
}
