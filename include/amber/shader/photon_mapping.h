/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
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
  rendering(
    Scene const& scene,
    camera_type const& camera,
    photon_map_type const& photon_map,
    size_t x, size_t y,
    DefaultSampler<>& sampler
  ) const
  {
    ray_type ray;
    radiant_type weight;
    std::tie(ray, weight, std::ignore, std::ignore) =
      camera.GenerateRay(x, y, &sampler);

    return estimatePower(scene, photon_map, ray, weight, sampler);
  }

  radiant_type
  estimatePower(
    Scene const& scene,
    photon_map_type const& photon_map,
    ray_type const& ray,
    radiant_type const& weight,
    DefaultSampler<>& sampler,
    size_t depth = 0
  ) const
  {
    hit_type hit;
    Object object;

    std::tie(hit, object) = scene.cast(ray);
    if (!hit) {
      return radiant_type();
    }

    if (object.surfaceType() == material::SurfaceType::Light) {
      if (dot(hit.normal, ray.direction) < 0) {
        return weight * object.emittance();
      } else {
        return radiant_type();
      }
    }

    if (object.surfaceType() == material::SurfaceType::Diffuse) {
      auto const photons = photon_map.kNearestNeighbours(hit.position,
        static_cast<real_type>(1),
        k_nearest_photon_
      );
      return weight * filter(photons, hit, object, -ray.direction);
    }

    if (depth > 10) { // FIXME rewrite with Russian roulette
      return radiant_type();
    }

    radiant_type power;

    auto const scatters = object.distributionLight(-ray.direction, hit.normal);
    for (auto const& scatter : scatters) {
      power += estimatePower(
        scene,
        photon_map,
        ray_type(hit.position, scatter.direction),
        weight * scatter.weight,
        sampler,
        depth + 1
      );
    }

    return power;
  }

  radiant_type
  filter(
    std::vector<photon_type> const& photons,
    hit_type const& hit,
    Object const& object,
    vector3_type const& direction_o
  ) const
  {
    // TODO sum up simply for now
    auto const squared_max_distance =
      (photons.back().position.template cast<real_type>() - hit.position)
      .squaredLength();

    radiant_type power;
    for (auto const& photon : photons) {
      auto const direction_i = photon.direction.template cast<real_type>();
      auto const bsdf = object.bsdf(direction_i, direction_o, hit.normal);
      power += bsdf * photon.power;
    }
    return power / kPI / squared_max_distance;
  }
};

}
}
