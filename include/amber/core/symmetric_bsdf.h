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

#include "core/material.h"

namespace amber {
namespace core {

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
}
