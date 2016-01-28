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

#include <algorithm>
#include <iterator>
#include <vector>

#include "amber/prelude/sampling.h"
#include "amber/rendering/leading.h"
#include "amber/scene/object.h"

namespace amber {
namespace scene {

template <typename Radiant>
class LightSet
{
public:
  LightSet(std::vector<Object<Radiant>>&& objects);

  std::tuple<const Object<Radiant>*, rendering::Leading<Radiant>>
  GenerateRay(Sampler& sampler) const;

  const real_type
  PDFArea(const Object<Radiant>& object) const noexcept;

  const real_type
  PDFDirection(
    const Object<Radiant>& object,
    const Ray& ray
  ) const noexcept;

private:
  std::vector<Object<Radiant>> objects_;
  std::vector<real_type> powers_;
};





template <typename Radiant>
LightSet<Radiant>::LightSet(std::vector<Object<Radiant>>&& objects)
: objects_()
, powers_()
{
  std::sort(
    objects.begin(),
    objects.end(),
    [](const auto& x, const auto& y){ return Power(x) < Power(y); }
  );

  objects_.reserve(objects.size());
  powers_.reserve(objects.size());

  real_type power = 0;
  for (const auto& object : objects) {
    power += Power(object);
    objects_.emplace_back(object);
    powers_.emplace_back(power);
  }
}

template <typename Radiant>
std::tuple<const Object<Radiant>*, rendering::Leading<Radiant>>
LightSet<Radiant>::GenerateRay(Sampler& sampler) const
{
  const auto pos = std::distance(powers_.begin(), std::lower_bound(
    powers_.begin(),
    powers_.end(),
    prelude::Uniform(powers_.back(), sampler)
  ));
  const auto& object = objects_.at(pos);
  const auto ray = object.SampleSurfacePoint(sampler);

  return std::make_tuple(
    &object,
    rendering::Leading<Radiant>(
      ray.Origin(),
      ray.Direction(),
      prelude::HemispherePSA(ray.Direction(), sampler),
      object.Irradiance() / PDFArea(object)
    )
  );
}

template <typename Radiant>
const real_type
LightSet<Radiant>::PDFArea(const Object<Radiant>& object) const noexcept
{
  return Sum(object.Irradiance()) / powers_.back();
}


template <typename Radiant>
const real_type
LightSet<Radiant>::PDFDirection(
  const Object<Radiant>& object,
  const Ray& ray
) const noexcept
{
  // FIXME expect only diffuse lights
  return 1 / kPI;
}

}
}
