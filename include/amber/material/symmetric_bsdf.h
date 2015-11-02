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
  lightScatterPDF(vector3_type const& direction_i,
                  vector3_type const& direction_o,
                  vector3_type const& normal) const noexcept {
    return scatterPDF(direction_i, direction_o, normal);
  }

  radiant_value_type
  importanceScatterPDF(vector3_type const& direction_i,
                       vector3_type const& direction_o,
                       vector3_type const& normal) const noexcept {
    return scatterPDF(direction_i, direction_o, normal);
  }

  virtual radiant_value_type
  scatterPDF(vector3_type const&,
             vector3_type const&,
             vector3_type const&) const noexcept = 0;

  scatter_type
  sampleLightScatter(vector3_type const& direction_i,
                     vector3_type const& normal,
                     Sampler* sampler) const {
    return sampleScatter(direction_i, normal, sampler);
  }

  scatter_type
  sampleImportanceScatter(vector3_type const& direction_i,
                          vector3_type const& normal,
                          Sampler* sampler) const {
    return sampleScatter(direction_i, normal, sampler);
  }

  virtual scatter_type
  sampleScatter(vector3_type const&,
                vector3_type const&,
                Sampler*) const = 0;

  std::vector<scatter_type>
  specularLightScatters(vector3_type const& direction_i,
                        vector3_type const& normal) const {
    return specularScatters(direction_i, normal);
  }

  std::vector<scatter_type>
  specularImportanceScatters(vector3_type const& direction_i,
                             vector3_type const& normal) const {
    return specularScatters(direction_i, normal);
  }

  virtual std::vector<scatter_type>
  specularScatters(vector3_type const&,
                   vector3_type const&) const {
    throw std::logic_error("specularScatters is not implemented");
  }
};

}
}
