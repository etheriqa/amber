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

#include <memory>

#include "acceleration.h"
#include "component/light_sampler.h"

namespace amber {

template <
  typename Acceleration,
  typename Object = typename Acceleration::object_type,
  typename LightSampler = typename component::LightSampler<Object>
>
class Scene : public amber::Acceleration<Object>
{
public:
  using object_type        = Object;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using vector3_type       = typename Object::vector3_type;

  using acceleration_ptr   = std::shared_ptr<Acceleration const>;
  using light_sampler_ptr  = std::shared_ptr<LightSampler const>;

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

  void Write(std::ostream& os) const noexcept
  {
    os << "Scene()";
  }

  std::tuple<hit_type, Object> Cast(ray_type const& ray) const noexcept
  {
    return acceleration_->Cast(ray);
  }

  bool TestVisibility(ray_type const& ray, Object const& object) const noexcept
  {
    return acceleration_->TestVisibility(ray, object);
  }

  radiant_value_type
  LightPDFArea(Object const& object) const noexcept
  {
    return light_sampler_->PDFArea(object);
  }

  std::tuple<ray_type, radiant_type, Object, radiant_value_type, vector3_type>
  GenerateLightRay(Sampler* sampler) const
  {
    return light_sampler_->GenerateLightRay(sampler);
  }
};

}
