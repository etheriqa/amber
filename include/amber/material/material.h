/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <vector>

#include "base/sampler.h"
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
  virtual bool isEmissive() const noexcept = 0;
  virtual Radiant emittance() const noexcept = 0;

  virtual Radiant
  bsdf(vector3_type const&,
       vector3_type const&,
       vector3_type const&) const noexcept = 0;

  virtual radiant_value_type
  lightScatterPDF(vector3_type const&,
                  vector3_type const&,
                  vector3_type const&) const noexcept = 0;

  virtual radiant_value_type
  importanceScatterPDF(vector3_type const&,
                       vector3_type const&,
                       vector3_type const&) const noexcept = 0;

  virtual scatter_type
  sampleLightScatter(vector3_type const&,
                     vector3_type const&,
                     Sampler*) const = 0;

  virtual scatter_type
  sampleImportanceScatter(vector3_type const&,
                          vector3_type const&,
                          Sampler*) const = 0;

  virtual std::vector<scatter_type>
  specularLightScatters(vector3_type const&,
                        vector3_type const&) const {
    throw std::logic_error("specularLightScatters it not implemented");
  }

  virtual std::vector<scatter_type>
  specularImportanceScatters(vector3_type const&,
                             vector3_type const&) const {
    throw std::logic_error("specularImportanceScatters it not implemented");
  }
};

}
}
