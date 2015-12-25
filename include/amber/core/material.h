// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <algorithm>
#include <numeric>
#include <vector>

#include "core/sampler.h"
#include "core/scatter.h"
#include "core/surface_type.h"

namespace amber {
namespace core {

template <typename Radiant, typename RealType>
class Material
{
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = Scatter<Radiant, RealType>;
  using unit_vector3_type  = UnitVector3<RealType>;

public:
  virtual ~Material() {}

  virtual SurfaceType Surface() const noexcept = 0;

  virtual
  Radiant const
  Irradiance() const noexcept { return Radiant(); }

  virtual
  Radiant const
  Radiance(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept { return Radiant(); }

  virtual
  Radiant const
  BSDF(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept = 0;

  virtual
  Radiant const
  AdjointBSDF(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept = 0;

  virtual
  radiant_value_type const
  PDFLight(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept = 0;

  virtual
  radiant_value_type const
  PDFImportance(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept = 0;

  virtual
  scatter_type
  SampleLight(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal,
    Sampler& sampler
  ) const = 0;

  virtual
  scatter_type
  SampleImportance(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal,
    Sampler& sampler
  ) const = 0;

  virtual
  std::vector<scatter_type>
  DistributionLight(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const
  {
    throw std::logic_error("DistributionLight it not implemented");
  }

  virtual
  std::vector<scatter_type>
  DistributionImportance(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const
  {
    throw std::logic_error("DistributionImportance it not implemented");
  }
};

}
}
