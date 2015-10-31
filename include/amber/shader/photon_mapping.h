/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <iterator>
#include <mutex>
#include <thread>
#include <vector>

#include "shader/framework/photon_mapping.h"
#include "shader/shader.h"

namespace amber {
namespace shader {

template <typename Scene,
          typename Object = typename Scene::object_type>
class PhotonMapping : public Shader<Scene> {
private:
  using camera_type        = typename Shader<Scene>::camera_type;
  using image_type         = typename Shader<Scene>::image_type;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using real_type          = typename Object::real_type;
  using vector3_type       = typename Object::vector3_type;

  using pm_type            = typename framework::PhotonMapping<Scene>;

  using photon_map_type    = typename pm_type::photon_map_type;
  using photon_type        = typename pm_type::photon_type;

  size_t n_thread_, n_photon_, k_nearest_photon_;
  Progress progress_;

public:
  PhotonMapping(size_t n_thread, size_t n_photon, size_t k_nearest_photon)
    : n_thread_(n_thread),
      n_photon_(n_photon),
      k_nearest_photon_(k_nearest_photon),
      progress_(3) {}

  void write(std::ostream& os) const noexcept {
    os
      << "PhotonMapping(n_thread=" << n_thread_
      << ", n_photon=" << n_photon_
      << ", k_nearest_photon=" << k_nearest_photon_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(Scene const& scene, camera_type const& camera) {
    pm_type pm(scene);
    std::vector<std::thread> threads;
    std::mutex mtx;

    progress_.phase = "Photon Tracing";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = n_photon_;

    std::vector<photon_type> photons;
    for (size_t i = 0; i < n_thread_; i++) {
      threads.emplace_back([&](){
        std::vector<photon_type> buffer;
        DefaultSampler<> sampler((std::random_device()()));
        while (progress_.current_job++ < progress_.total_job) {
          pm.photonTracing(n_photon_, std::back_inserter(buffer), &sampler);
        }
        std::lock_guard<std::mutex> lock(mtx);
        std::move(buffer.begin(), buffer.end(), std::back_inserter(photons));
      });
    }
    while (!threads.empty()) {
      threads.back().join();
      threads.pop_back();
    }

    progress_.phase = "Photon Map Building";
    progress_.current_phase = 2;
    progress_.current_job = 0;
    progress_.total_job = 0;

    auto const photon_map = pm.buildPhotonMap(photons.begin(), photons.end());

    progress_.phase = "Distributed Ray Tracing";
    progress_.current_phase = 3;
    progress_.current_job = 0;
    progress_.total_job = camera.imageSize();

    std::vector<size_t> pixels(camera.imageSize());
    std::iota(pixels.begin(), pixels.end(), 0);
    std::shuffle(pixels.begin(), pixels.end(), std::random_device());

    image_type image(camera.imageWidth(), camera.imageHeight());
    for (size_t i = 0; i < n_thread_; i++) {
      threads.emplace_back([&](){
        DefaultSampler<> sampler((std::random_device()()));
        for (size_t j = progress_.current_job++;
             j < progress_.total_job;
             j = progress_.current_job++) {
          auto const x = pixels.at(j) % camera.imageWidth();
          auto const y = pixels.at(j) / camera.imageWidth();
          auto const power =
            rendering(scene, camera, photon_map, x, y, sampler);
          std::lock_guard<std::mutex> lock(mtx);
          image.at(x, y) += power;
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
  rendering(Scene const& scene,
            camera_type const& camera,
            photon_map_type const& photon_map,
            size_t x, size_t y,
            DefaultSampler<>& sampler) const {
    return estimatePower(scene,
                         photon_map,
                         camera.sampleFirstRay(x, y, &sampler),
                         radiant_type(1),
                         sampler);
  }

  radiant_type
  estimatePower(Scene const& scene,
                photon_map_type const& photon_map,
                ray_type const& ray,
                radiant_type const& weight,
                DefaultSampler<>& sampler,
                size_t depth = 0) const {
    radiant_type power;
    hit_type hit;
    Object object;

    std::tie(hit, object) = scene.acceleration()->cast(ray);
    if (!hit) {
      return radiant_type();
    }

    if (object.isEmissive() && dot(hit.normal, ray.direction) < 0) {
      power += weight * object.emittance();
    }

    if (object.surfaceType() == material::SurfaceType::diffuse) {
      auto const photons =
        photon_map.kNearestNeighbours(hit.position,
                                      static_cast<real_type>(1),
                                      k_nearest_photon_);
      return
        power + weight * gaussianFilter(photons, hit, object, -ray.direction);
    }

    if (depth > 10) { // FIXME rewrite with Russian roulette
      return power;
    }

    auto const samples = object.specularScatters(-ray.direction, hit.normal);
    auto const accumulator =
      [](auto const& acc, auto const& s){ return acc + s.psa_probability; };
    auto const p = std::accumulate(samples.begin(),
                                   samples.end(),
                                   static_cast<radiant_value_type>(0),
                                   accumulator);
    for (auto const& sample : samples) {
      power += estimatePower(scene,
                             photon_map,
                             ray_type(hit.position, sample.direction_o),
                             weight * sample.bsdf / sample.psa_probability,
                             sampler,
                             depth + 1) * sample.psa_probability / p;
    }

    return power;
  }

  radiant_type
  gaussianFilter(std::vector<photon_type> const& photons,
                 hit_type const& hit,
                 Object const& object,
                 vector3_type const& direction_o) const {
    const real_type alpha = 0.918;
    const real_type beta = 1.953;

    auto const squared_max_distance =
      (photons.back().position.template cast<real_type>() - hit.position)
      .squaredLength();

    radiant_type power;
    for (auto const& photon : photons) {
      auto const squared_distance =
        (photon.position.template cast<real_type>() - hit.position)
        .squaredLength();
      auto const weight = 1 -
        (1 - std::exp(-beta * squared_distance / 2 / squared_max_distance)) /
        (1 - std::exp(-beta));
      auto const bsdf =
        object.bsdf(photon.direction.template cast<real_type>(),
                    direction_o,
                    hit.normal);
      power += bsdf * photon.power * weight;
    }
    return power * alpha / kPI / squared_max_distance;
  }
};

}
}
