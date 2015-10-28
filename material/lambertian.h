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
class Lambertian : public Material<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

private:
  Radiant kd_;

public:
  explicit Lambertian(Radiant const& kd) noexcept : kd_(kd) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::diffuse; }

  Radiant bsdf(vector3_type const& direction_i,
               vector3_type const& direction_o,
               vector3_type const& normal) const noexcept {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return Radiant();
    } else {
      return kd_ / kPI;
    }
  }

  radiant_value_type pdf(vector3_type const& direction_i,
                         vector3_type const& direction_o,
                         vector3_type const& normal) const noexcept {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return radiant_value_type();
    } else {
      return 1 / kPI;
    }
  }

  scatter_type sampleScatter(Radiant const&,
                             vector3_type const& direction_i,
                             vector3_type const& normal,
                             Sampler* sampler) const {
    auto const w = dot(direction_i, normal) > 0 ? normal : -normal;
    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = sampler->hemispherePSA(w);

    return scatter_type(direction_o, kd_ / kPI, 1 / kPI);
  }
};

}
}
