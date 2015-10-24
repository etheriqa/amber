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
#include "material/surface_type.h"
#include "random.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Material {
public:
  using material_type      = Material<Radiant, RealType>;
  using radiant_type       = Radiant;
  using radiant_value_type = typename Radiant::value_type;
  using real_type          = RealType;
  using vector3_type       = geometry::Vector3<RealType>;

  struct ScatteringSample
  {
    vector3_type direction_o;
    radiant_type bsdf;
    real_type psa_probability;
  };

  virtual SurfaceType surfaceType() const noexcept = 0;
  virtual bool isEmissive() const noexcept { return false; }
  virtual radiant_type emittance() const noexcept { return radiant_type(); }

  virtual radiant_type bsdf(const vector3_type&,
                            const vector3_type&,
                            const vector3_type&) const noexcept {
    return radiant_type();
  }

  virtual ScatteringSample sampleScattering(const radiant_type& radiant,
                                            const vector3_type& direction_i,
                                            const vector3_type& normal,
                                            Random& random) const {
    const auto candidates = scatteringCandidates(radiant, direction_i, normal);
    if (candidates.size() == 1) {
      return candidates.front();
    }

    const auto accumulator = [](const auto& acc, const auto& sample){
      return acc + sample.psa_probability;
    };
    const auto r =
      random.uniform(std::accumulate(candidates.begin(),
                                     candidates.end(),
                                     static_cast<radiant_value_type>(0),
                                     accumulator));
    real_type p = 0;
    for (const auto& s : candidates) {
      p += s.psa_probability;
      if (r < p) {
        return s;
      }
    }
    return candidates.back();
  }

  virtual std::vector<ScatteringSample>
  scatteringCandidates(const radiant_type&,
                       const vector3_type&,
                       const vector3_type&) const {
    throw std::logic_error("scatteringCandidates is not implemented");
  }
};

}
}
