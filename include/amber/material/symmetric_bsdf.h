/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class SymmetricBSDF : public Material<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = Scatter<Radiant, RealType>;
  using vector3_type       = geometry::Vector3<RealType>;

  radiant_value_type
  pdfLight(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    return pdf(direction_i, direction_o, normal);
  }

  radiant_value_type
  pdfImportance(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    return pdf(direction_i, direction_o, normal);
  }

  scatter_type
  sampleLight(
    vector3_type const& direction_i,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return sample(direction_i, normal, sampler);
  }

  scatter_type
  sampleImportance(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const {
    return sample(direction_o, normal, sampler);
  }

  std::vector<scatter_type>
  distributionLight(
    vector3_type const& direction_i,
    vector3_type const& normal
  ) const
  {
    return distribution(direction_i, normal);
  }

  std::vector<scatter_type>
  distributionImportance(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    return distribution(direction_o, normal);
  }

protected:
  virtual
  radiant_value_type
  pdf(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept = 0;

  virtual
  scatter_type
  sample(
    vector3_type const& direction_i,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return this->sampleScatter(distribution(direction_i, normal), sampler);
  }

  virtual
  std::vector<scatter_type>
  distribution(
    vector3_type const&,
    vector3_type const&
  ) const
  {
    throw std::logic_error("specularScatters is not implemented");
  }
};

}
}
