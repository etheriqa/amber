// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include <memory>

#include "amber/scene/material_basic.h"

namespace amber {
namespace scene {

template <typename Radiant>
std::unique_ptr<Material<Radiant>>
MakeLambertian(const Radiant& kd);

class BasicLambertian
{
public:
  rendering::SurfaceType Surface() const noexcept;

  real_type
  SymmetricBSDF(
    const UnitVector3& direction_i,
    const UnitVector3& direction_o,
    const UnitVector3& normal
  ) const noexcept;

  real_type
  PDF(
    const UnitVector3& direction_i,
    const UnitVector3& direction_o,
    const UnitVector3& normal
  ) const noexcept;

  BasicScatter
  Sample(
    const UnitVector3& direction_o,
    const UnitVector3& normal,
    Sampler& sampler
  ) const;
};





template <typename Radiant>
std::unique_ptr<Material<Radiant>>
MakeLambertian(const Radiant& kd)
{
  return
    std::make_unique<
      SymmetricBasicMaterialForwarder<Radiant, BasicLambertian>
    >
    (BasicLambertian(), kd);
}

}
}
