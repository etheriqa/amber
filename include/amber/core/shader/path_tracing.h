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

template <typename Object>
class PathTracing : public Shader<Object>
{
private:
  using camera_type        = typename Shader<Object>::camera_type;
  using image_type         = typename Shader<Object>::image_type;
  using scene_type         = typename Shader<Object>::scene_type;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using real_type          = typename Object::real_type;
  using vector3_type       = typename Object::vector3_type;

  size_t n_threads_, spp_;
  Progress progress_;

public:
  PathTracing(size_t n_threads, size_t spp) noexcept
  : n_threads_(n_threads),
    spp_(spp),
    progress_(1)
  {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "PathTracing(n_threads=" << n_threads_
      << ", spp=" << spp_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(scene_type const& scene, camera_type const& camera)
  {
    progress_.phase = "Path Tracing";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = spp_;

    std::vector<std::thread> threads;
    std::mutex mtx;
    image_type image(camera.imageWidth(), camera.imageHeight());
    for (size_t i = 0; i < n_threads_; i++) {
      threads.emplace_back([&](){
        DefaultSampler<> sampler((std::random_device()()));
        image_type buffer(camera.imageWidth(), camera.imageHeight());
        while (progress_.current_job++ < progress_.total_job) {
          for (size_t y = 0; y < camera.imageHeight(); y++) {
            for (size_t x = 0; x < camera.imageWidth(); x++) {
              buffer.at(x, y) += Sample(scene, camera, x, y, sampler);
            }
          }
        }
        std::lock_guard<std::mutex> lock(mtx);
        for (size_t y = 0; y < camera.imageHeight(); y++) {
          for (size_t x = 0; x < camera.imageWidth(); x++) {
            image.at(x, y) += buffer.at(x, y) / spp_;
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
  radiant_type
  Sample(
    scene_type const& scene,
    camera_type const& camera,
    size_t x,
    size_t y,
    DefaultSampler<>& sampler
  ) const
  {
    ray_type ray;
    radiant_type weight;
    std::tie(ray, weight, std::ignore, std::ignore) =
      camera.GenerateRay(x, y, &sampler);

    radiant_type power;

    for (;;) {
      hit_type hit;
      Object object;
      std::tie(hit, object) = scene.Cast(ray);
      if (!hit) {
        break;
      }

      if (object.Surface() == SurfaceType::Light &&
          Dot(hit.normal, ray.direction) < 0) {
        power += weight * object.Radiance();
      }

      auto const scatter =
        object.SampleLight(-ray.direction, hit.normal, &sampler);
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, Max(scatter.weight));

      if (sampler.uniform<radiant_value_type>() >= p_russian_roulette) {
        break;
      }

      ray = ray_type(hit.position, scatter.direction);
      weight *= scatter.weight / p_russian_roulette;
    }

    return power;
  }
};

}
}
}
