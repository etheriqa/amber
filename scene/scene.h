/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
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
class Scene {
public:
  using object_type       = Object;

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

  acceleration_ptr const& acceleration() const noexcept {
    return acceleration_;
  }

  light_sampler_ptr const& light_sampler() const noexcept {
    return light_sampler_;
  }
};

}
}
