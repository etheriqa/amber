/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "material.h"

namespace amber {

template <typename Radiant, typename RealType>
class SymmetricBSDF : public Material<Radiant, RealType> {
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = Scatter<Radiant, RealType>;
  using vector3_type       = Vector3<RealType>;

  radiant_value_type
  PDFLight(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    return pdf(direction_i, direction_o, normal);
  }

  radiant_value_type
  PDFImportance(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    return pdf(direction_i, direction_o, normal);
  }

  scatter_type
  SampleLight(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return Sample(direction_o, normal, sampler);
  }

  scatter_type
  SampleImportance(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const {
    return Sample(direction_o, normal, sampler);
  }

  std::vector<scatter_type>
  DistributionLight(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    return distribution(direction_o, normal);
  }

  std::vector<scatter_type>
  DistributionImportance(
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
  Sample(
    vector3_type const& direction_o,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return this->SampleScatter(distribution(direction_o, normal), sampler);
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
