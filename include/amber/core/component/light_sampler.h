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
#include <vector>

#include "core/sampler.h"
#include "core/surface_type.h"

namespace amber {
namespace core {
namespace component {

template <
  typename Object,
  typename Radiant = typename Object::radiant_type
>
class LightSampler
{
public:
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using vector3_type       = typename Object::vector3_type;

private:

  struct Node
  {
    radiant_value_type partial_sum_power;
    Object object;

    Node(radiant_value_type partial_sum_power, Object const& object) noexcept
    : partial_sum_power(partial_sum_power), object(object) {}

    bool operator<(const Node& node) const noexcept
    {
      return partial_sum_power < node.partial_sum_power;
    }
  };

  std::vector<Node> nodes_;
  radiant_value_type total_power_;

public:
  template <typename InputIterator>
  LightSampler(InputIterator first, InputIterator last) noexcept
  {
    std::for_each(first, last, [&](auto const& object){
      if (object.Surface() != SurfaceType::Light) {
        return;
      }
      total_power_ += Sum(object.power());
      nodes_.emplace_back(total_power_, object);
    });
  }

  Object
  SampleLight(Sampler& sampler) const
  {
    return std::lower_bound(
      nodes_.begin(),
      nodes_.end(),
      Node(Uniform(total_power_, sampler), Object())
    )->object;
  }

  radiant_value_type
  PDFArea(Object const& object) const noexcept
  {
    return Sum(object.Radiance()) * kPI / total_power_;
  }

  std::tuple<ray_type, Radiant, Object, radiant_value_type, vector3_type>
  GenerateLightRay(Sampler& sampler) const
  {
    auto const light = SampleLight(sampler);
    auto const ray = light.SamplePoint(sampler);
    auto const p_area = PDFArea(light);
    return std::make_tuple(
      ray_type(ray.origin, std::get<0>(HemispherePSA(ray.direction, sampler))),
      light.Radiance() * kPI / p_area,
      light,
      p_area,
      ray.direction
    );
  }
};

}
}
}
