/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <numeric>
#include <vector>

#include "sampler.h"
#include "material/scatter.h"
#include "material/surface_type.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Material {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = Scatter<Radiant, RealType>;
  using vector3_type       = geometry::Vector3<RealType>;

  using radiant_type       = Radiant;                      // TODO remove

public:
  virtual SurfaceType surfaceType() const noexcept = 0;
  virtual Radiant emittance() const noexcept = 0;

  virtual
  Radiant
  bsdf(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept = 0;

  virtual
  radiant_value_type
  pdfLight(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept = 0;

  virtual
  radiant_value_type
  pdfImportance(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept = 0;

  virtual
  scatter_type
  sampleLight(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return sampleScatter(distributionLight(direction_o, normal), sampler);
  }

  virtual
  scatter_type
  sampleImportance(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return sampleScatter(distributionImportance(direction_o, normal), sampler);
  }

  virtual
  std::vector<scatter_type>
  distributionLight(
    vector3_type const&,
    vector3_type const&
  ) const
  {
    throw std::logic_error("distributionLight it not implemented");
  }

  virtual
  std::vector<scatter_type>
  distributionImportance(
    vector3_type const&,
    vector3_type const&
  ) const
  {
    throw std::logic_error("distributionImportance it not implemented");
  }

protected:
  static
  scatter_type
  sampleScatter(
    std::vector<scatter_type> const& scatters,
    Sampler* sampler
  )
  {
    if (scatters.empty()) {
      throw std::logic_error("no scatter event given");
    }

    if (scatters.size() == 1) {
      return scatters.front();
    }

    std::vector<radiant_value_type> weights(scatters.size());
    std::transform(
      scatters.begin(),
      scatters.end(),
      weights.begin(),
      [](auto const& scatter){ return scatter.weight.sum(); }
    );
    std::partial_sum(weights.begin(), weights.end(), weights.begin());

    auto const distance = std::distance(
      weights.begin(),
      std::lower_bound(
        weights.begin(),
        weights.end(),
        sampler->uniform(weights.back())
    ));

    auto const& scatter = scatters.at(distance);
    return scatter_type(
      scatter.direction,
      scatter.weight / scatter.weight.sum() * weights.back()
    );
  }
};

}
}
