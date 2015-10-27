/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Light : public Material<Radiant, RealType> {
public:
  using scatter_type = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant radiance_;

public:
  explicit Light(Radiant const& radiance) noexcept : radiance_(radiance) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::diffuse; }
  bool isEmissive() const noexcept { return true; }
  Radiant emittance() const noexcept { return radiance_; }

  scatter_type sampleScatter(Radiant const&,
                             vector3_type const&,
                             vector3_type const&,
                             Sampler*) const {
    return scatter_type(vector3_type(), Radiant(), 1);
  }
};

}
}
