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
#include <mutex>
#include <vector>

#include "core/shader.h"
#include "core/surface_type.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class LightTracing : public Shader<Object>
{
private:
  using typename Shader<Object>::camera_type;
  using typename Shader<Object>::image_type;
  using typename Shader<Object>::scene_type;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using real_type          = typename Object::real_type;
  using unit_vector3_type  = typename Object::unit_vector3_type;

public:
  LightTracing() noexcept {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "LightTracing()";
  }

  image_type
  operator()(
    scene_type const& scene,
    camera_type const& camera,
    Context& ctx
  )
  {
    image_type image(camera.ImageWidth(), camera.ImageHeight());

    {
      std::mutex mtx;

      IterateParallel(ctx, [&](auto const&){
        MTSampler sampler((std::random_device()()));

        image_type buffer(camera.ImageWidth(), camera.ImageHeight());
        for (std::size_t j = 0; j < camera.ImageSize(); j++) {
          Sample(scene, camera, buffer, sampler);
        }

        std::lock_guard<std::mutex> lock(mtx);
        image += buffer;
      });
    }

    for (std::size_t y = 0; y < camera.ImageHeight(); y++) {
      for (std::size_t x = 0; x < camera.ImageWidth(); x++) {
        image.at(x, y) /= ctx.IterationCount();
      }
    }

    return image;
  }

private:
  void
  Sample(
    scene_type const& scene,
    camera_type const& camera,
    image_type& image,
    Sampler& sampler
  ) const
  {
    ray_type ray;
    radiant_type weight;
    std::tie(ray, weight, std::ignore, std::ignore, std::ignore, std::ignore) =
      scene.GenerateLightRay(sampler);

    weight /= camera.ImageSize();

    for (;;) {
      hit_type hit;
      Object object;
      std::tie(hit, object) = scene.Cast(ray);
      if (!hit) {
        break;
      }

      if (object.Surface() == SurfaceType::Eye) {
        auto const response = camera.Response(-ray.direction, hit.position);
        if (response) {
          auto const& x = std::get<0>(*response);
          auto const& y = std::get<1>(*response);
          auto const& sensor_weight = std::get<2>(*response);
          image.at(x, y) += weight * sensor_weight;
        }
      }

      auto const scatter =
        object.SampleImportance(-ray.direction, hit.normal, sampler);
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, Max(scatter.weight));

      if (Uniform<radiant_value_type>(sampler) >= p_russian_roulette) {
        break;
      }

      ray = ray_type(hit.position, scatter.direction);
      weight *= scatter.weight / p_russian_roulette;
    }
  }
};

}
}
}
