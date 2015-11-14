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
#include <thread>
#include <vector>

#include "core/shader.h"
#include "core/surface_type.h"

namespace amber {
namespace core {
namespace shader {

template <
  typename Scene,
  typename Object = typename Scene::object_type
>
class LightTracing : public Shader<Scene>
{
private:
  using camera_type        = typename Shader<Scene>::camera_type;
  using image_type         = typename Shader<Scene>::image_type;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using real_type          = typename Object::real_type;
  using vector3_type       = typename Object::vector3_type;

  size_t n_threads_, n_samples_;
  Progress progress_;

public:
  LightTracing(size_t n_threads, size_t n_samples) noexcept
  : n_threads_(n_threads),
    n_samples_(n_samples),
    progress_(1)
  {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "LightTracing(n_threads=" << n_threads_
      << ", n_samples=" << n_samples_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(Scene const& scene, camera_type const& camera)
  {
    progress_.phase = "Light Tracing";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = n_samples_;

    std::vector<std::thread> threads;
    std::mutex mtx;
    image_type image(camera.imageWidth(), camera.imageHeight());
    for (size_t i = 0; i < n_threads_; i++) {
      threads.emplace_back([&](){
        DefaultSampler<> sampler((std::random_device()()));
        image_type buffer(camera.imageWidth(), camera.imageHeight());
        while (progress_.current_job++ < progress_.total_job) {
          Sample(scene, camera, buffer, sampler);
        }
        std::lock_guard<std::mutex> lock(mtx);
        for (size_t y = 0; y < camera.imageHeight(); y++) {
          for (size_t x = 0; x < camera.imageWidth(); x++) {
            image.at(x, y) += buffer.at(x, y) / n_samples_;
          }
        }
      });
    }
    while (!threads.empty()) {
      threads.back().join();
      threads.pop_back();
    }
    return image;
  }

private:
  void
  Sample(
    Scene const& scene,
    camera_type const& camera,
    image_type& image,
    DefaultSampler<>& sampler
  ) const
  {
    ray_type ray;
    radiant_type weight;
    std::tie(ray, weight, std::ignore, std::ignore, std::ignore) =
      scene.GenerateLightRay(&sampler);

    for (;;) {
      hit_type hit;
      Object object;
      std::tie(hit, object) = scene.Cast(ray);
      if (!hit) {
        break;
      }

      if (object.Surface() == SurfaceType::Eye &&
          Dot(ray.direction, hit.normal) < 0) {
        auto const point = camera.ResponsePoint(ray.direction, hit.position);
        if (point) {
          image.at(std::get<0>(*point), std::get<1>(*point)) += weight;
        }
      }

      auto const scatter =
        object.SampleImportance(-ray.direction, hit.normal, &sampler);
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, Max(scatter.weight));

      if (sampler.uniform<radiant_value_type>() >= p_russian_roulette) {
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
