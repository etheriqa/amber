/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <mutex>
#include <thread>
#include <vector>

#include "shader/shader.h"

namespace amber {
namespace shader {

template <typename Scene,
          typename Object = typename Scene::object_type>
class PathTracing : public Shader<Scene> {
private:
  using camera_type        = typename Shader<Scene>::camera_type;
  using image_type         = typename Shader<Scene>::image_type;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using real_type          = typename Object::real_type;
  using vector3_type       = typename Object::vector3_type;

  size_t n_thread_, spp_;
  Progress progress_;

public:
  PathTracing(size_t n_thread, size_t spp) noexcept
    : n_thread_(n_thread),
      spp_(spp),
      progress_(1) {}

  void write(std::ostream& os) const noexcept {
    os
      << "PathTracing(n_thread=" << n_thread_
      << ", spp=" << spp_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(Scene const& scene, camera_type const& camera) {
    progress_.phase = "Path Tracing";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = spp_;

    std::vector<std::thread> threads;
    std::mutex mtx;
    image_type image(camera.imageWidth(), camera.imageHeight());
    for (size_t i = 0; i < n_thread_; i++) {
      threads.emplace_back([&](){
        DefaultSampler<> sampler((std::random_device()()));
        image_type buffer(camera.imageWidth(), camera.imageHeight());
        while (progress_.current_job++ < progress_.total_job) {
          for (size_t y = 0; y < camera.imageHeight(); y++) {
            for (size_t x = 0; x < camera.imageWidth(); x++) {
              buffer.at(x, y) +=
                samplePixel(scene, camera, x, y, sampler) /
                spp_;
            }
          }
        }
        std::lock_guard<std::mutex> lock(mtx);
        for (size_t y = 0; y < camera.imageHeight(); y++) {
          for (size_t x = 0; x < camera.imageWidth(); x++) {
            image.at(x, y) += buffer.at(x, y);
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
  radiant_type samplePixel(Scene const& scene,
                           camera_type const& camera,
                           size_t x,
                           size_t y,
                           DefaultSampler<>& sampler) const {
    radiant_type power, weight(1);
    ray_type ray = camera.sampleFirstRay(x, y, &sampler);
    hit_type hit;
    Object object;

    for (;;) {
      std::tie(hit, object) = scene.acceleration()->cast(ray);
      if (!hit) {
        break;
      }

      if (object.isEmissive() && dot(hit.normal, ray.direction) < 0) {
        power += weight * object.emittance();
      }

      auto const sample =
        object.sampleImportanceScatter(-ray.direction, hit.normal, &sampler);
      ray = ray_type(hit.position, sample.direction_o);
      auto const reflectance = sample.bsdf / sample.psa_probability;
      weight *= reflectance;

      auto const p_russian_roulette = reflectance.max();
      if (sampler.uniform<real_type>() >= p_russian_roulette) {
        break;
      }
      weight /= std::min<real_type>(1, p_russian_roulette);
    }

    return power;
  }
};

}
}
