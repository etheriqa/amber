/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Lambertian : public Material<Radiant, RealType> {
private:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

  radiant_type kd_;

public:
  explicit Lambertian(const radiant_type& kd) noexcept : kd_(kd) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::diffuse; }

  radiant_type bsdf(const vector3_type& direction_i,
                    const vector3_type& direction_o,
                    const vector3_type& normal) const noexcept {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return radiant_type();
    } else {
      return kd_ / kPI;
    }
  }

  scattering_sample_type sampleScattering(const radiant_type&,
                                          const vector3_type& direction_i,
                                          const vector3_type& normal,
                                          Random& random) const {
    const auto w = dot(direction_i, normal) > 0 ? normal : -normal;
    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = random.hemisphere_psa(w);

    scattering_sample_type sample;
    sample.direction_o = direction_o;
    sample.bsdf = kd_ / kPI;
    sample.psa_probability = 1 / kPI;
    return sample;
  }
};

}
}
