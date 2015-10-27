/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <numeric>
#include <vector>

#include "base/sampler.h"
#include "material/scatter.h"
#include "material/surface_type.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Material {
public:
  using scatter_type       = Scatter<Radiant, RealType>;
  using vector3_type       = geometry::Vector3<RealType>;

  using radiant_value_type = typename Radiant::value_type; // TODO remove
  using radiant_type       = Radiant;                      // TODO remove

public:
  virtual SurfaceType surfaceType() const noexcept = 0;
  virtual bool isEmissive() const noexcept { return false; }
  virtual Radiant emittance() const noexcept { return Radiant(); }

  virtual Radiant bsdf(vector3_type const&,
                       vector3_type const&,
                       vector3_type const&) const noexcept {
    return Radiant();
  }

  virtual scatter_type sampleScatter(Radiant const& radiant,
                                     vector3_type const& direction_i,
                                     vector3_type const& normal,
                                     Sampler* sampler) const {
    auto const scatters = specularScatters(radiant, direction_i, normal);
    if (scatters.size() == 1) {
      return scatters.front();
    }

    auto const accumulator = [](auto const& acc, auto const& scatter){
      return acc + scatter.psa_probability;
    };
    auto const r =
      sampler->uniform(std::accumulate(scatters.begin(),
                                       scatters.end(),
                                       static_cast<radiant_value_type>(0),
                                       accumulator));
    radiant_value_type p = 0;
    for (auto const& scatter : scatters) {
      p += scatter.psa_probability;
      if (r < p) {
        return scatter;
      }
    }
    return scatters.back();
  }

  virtual std::vector<scatter_type>
  specularScatters(Radiant const&,
                   vector3_type const&,
                   vector3_type const&) const {
    throw std::logic_error("specularScatters is not implemented");
  }
};

}
}
