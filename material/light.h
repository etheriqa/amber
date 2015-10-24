/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "constant.h"
#include "material/lambertian.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Light : public Material<Radiant, RealType> {
public:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

private:
  radiant_type radiance_;

public:
  explicit Light(const radiant_type& radiance) noexcept : radiance_(radiance) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::diffuse; }
  bool isEmissive() const noexcept { return true; }
  radiant_type emittance() const noexcept { return radiance_; }

  scattering_sample_type sampleScattering(const radiant_type&,
                                          const vector3_type& direction_i,
                                          const vector3_type& normal,
                                          Random& random) const {
    return Lambertian<radiant_type, real_type>(radiant_type())
      .sampleScattering(radiant_type(), direction_i, normal, random);
  }
};

}
}
