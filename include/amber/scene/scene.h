/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <memory>

#include "shader/framework/light_sampler.h"

namespace amber {
namespace scene {

template <
  typename Acceleration,
  typename Object = typename Acceleration::object_type,
  typename LightSampler = typename shader::framework::LightSampler<Object>>
class Scene : public acceleration::Acceleration<Object>
{
public:
  using hit_type          = typename Object::hit_type;
  using object_type       = Object;
  using ray_type          = typename Object::ray_type;

  using acceleration_ptr  = std::shared_ptr<Acceleration const>;
  using light_sampler_ptr = std::shared_ptr<LightSampler const>;

private:
  acceleration_ptr acceleration_;
  light_sampler_ptr light_sampler_;

public:
  template <typename InputIterator>
  Scene(InputIterator first, InputIterator last)
    : acceleration_(std::make_shared<Acceleration>(first, last)),
      light_sampler_(std::make_shared<LightSampler>(first, last)) {}

  acceleration_ptr const& acceleration() const noexcept
  {
    return acceleration_;
  }

  light_sampler_ptr const& light_sampler() const noexcept
  {
    return light_sampler_;
  }

  void write(std::ostream& os) const noexcept
  {
    os << "Scene()";
  }

  std::tuple<hit_type, Object> cast(ray_type const& ray) const noexcept
  {
    return acceleration_->cast(ray);
  }

  bool testVisibility(ray_type const& ray, Object const& object) const noexcept
  {
    return acceleration_->testVisibility(ray, object);
  }

  Object sampleLight(Sampler* sampler) const
  {
    return (*light_sampler_)(sampler);
  }
};

}
}
