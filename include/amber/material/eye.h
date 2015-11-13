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

#include "constant.h"
#include "symmetric_bsdf.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Eye : public SymmetricBSDF<Radiant, RealType>
{
public:
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = typename Material<Radiant, RealType>::scatter_type;
  using vector3_type       = typename Material<Radiant, RealType>::vector3_type;

  Eye() noexcept {}

  SurfaceType Surface() const noexcept
  {
    return amber::SurfaceType::Eye;
  }

  Radiant
  BSDF(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept
  {
    return Radiant();
  }

  radiant_value_type
  pdf(
    vector3_type const&,
    vector3_type const&,
    vector3_type const&
  ) const noexcept
  {
    return kDiracDelta;
  }

  std::vector<scatter_type>
  distribution(
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const
  {
    return {
      scatter_type(direction_o, Radiant(1)),
    };
  }
};

}
}
