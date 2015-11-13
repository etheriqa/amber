/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <vector>

#include "sampler.h"
#include "surface_type.h"

namespace amber {
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
      total_power_ += object.power().Sum();
      nodes_.emplace_back(total_power_, object);
    });
  }

  Object
  SampleLight(Sampler *sampler) const
  {
    return std::lower_bound(
      nodes_.begin(),
      nodes_.end(),
      Node(sampler->uniform(total_power_), Object())
    )->object;
  }

  radiant_value_type
  PDFArea(Object const& object) const noexcept
  {
    return object.Radiance().Sum() * kPI / total_power_;
  }

  std::tuple<ray_type, Radiant, Object, radiant_value_type, vector3_type>
  GenerateLightRay(Sampler* sampler) const
  {
    auto const light = SampleLight(sampler);
    auto const ray = light.SamplePoint(sampler);
    auto const p_area = PDFArea(light);
    return std::make_tuple(
      ray_type(ray.origin, std::get<0>(sampler->hemispherePSA(ray.direction))),
      light.Radiance() * kPI / p_area,
      light,
      p_area,
      ray.direction
    );
  }
};

}
}
