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
#include "scatter.h"
#include "surface_type.h"

namespace amber {

template <typename Radiant, typename RealType>
class Material
{
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = Scatter<Radiant, RealType>;
  using vector3_type       = Vector3<RealType>;

  using radiant_type       = Radiant;                      // TODO remove

public:
  virtual SurfaceType Surface() const noexcept = 0;

  virtual Radiant Radiance() const noexcept { return Radiant(); }

  virtual
  Radiant
  BSDF(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept = 0;

  virtual
  radiant_value_type
  PDFLight(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept = 0;

  virtual
  radiant_value_type
  PDFImportance(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept = 0;

  virtual
  scatter_type
  SampleLight(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return SampleScatter(DistributionLight(direction_o, normal), sampler);
  }

  virtual
  scatter_type
  SampleImportance(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return SampleScatter(DistributionImportance(direction_o, normal), sampler);
  }

  virtual
  std::vector<scatter_type>
  DistributionLight(
    vector3_type const&,
    vector3_type const&
  ) const
  {
    throw std::logic_error("distributionLight it not implemented");
  }

  virtual
  std::vector<scatter_type>
  DistributionImportance(
    vector3_type const&,
    vector3_type const&
  ) const
  {
    throw std::logic_error("distributionImportance it not implemented");
  }

protected:
  static
  scatter_type
  SampleScatter(
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
      [](auto const& scatter){ return scatter.weight.Sum(); }
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
      scatter.weight / scatter.weight.Sum() * weights.back()
    );
  }
};

}
