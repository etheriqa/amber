/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "shader/framework/photon_mapping.h"
#include "shader/shader.h"

namespace amber {
namespace shader {

template <typename Scene,
          typename Object = typename Scene::object_type>
class StochasticProgressivePhotonMapping : public Shader<Scene> {
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

  struct HitPoint
  {
    Object object;
    vector3_type position;
    vector3_type normal;
    vector3_type direction;
    radiant_type weight;

    HitPoint() noexcept
    : object(),
      position(),
      normal(),
      direction(),
      weight(1)
    {}
  };

  struct Statistics
  {
    size_t n_photons;
    radiant_type flux;
    real_type radius;
    std::atomic<bool> locked;

    Statistics() noexcept
    : n_photons(),
      flux(),
      radius(),
      locked(false)
    {}
  };

  size_t n_threads_, n_photons_, k_nearest_photons_, n_passes_;
  double initial_radius_, alpha_;
  Progress progress_;

public:
  StochasticProgressivePhotonMapping(
    size_t n_threads,
    size_t n_photons,
    size_t k_nearest_photons,
    size_t n_passes,
    double initial_radius,
    double alpha
  ) noexcept
  : n_threads_(n_threads),
    n_photons_(n_photons),
    k_nearest_photons_(k_nearest_photons),
    n_passes_(n_passes),
    initial_radius_(initial_radius),
    alpha_(alpha),
    progress_(1)
  {}

  void write(std::ostream& os) const noexcept
  {
    os
      << "StochasticProgressivePhotonMapping(n_threads=" << n_threads_
      << ", n_photons=" << n_photons_
      << ", k_nearest_photons=" << k_nearest_photons_
      << ", n_passes=" << n_passes_
      << ", initial_radius=" << initial_radius_
      << ", alpha=" << alpha_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(Scene const& scene, camera_type const& camera)
  {
    pm_type pm(scene);
    std::vector<std::thread> threads;

    progress_.phase = "Photon Pass + Distributed Ray Tracing Pass";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = n_passes_;

    std::vector<Statistics> statistics(camera.imageSize());
    for (auto& stats : statistics) {
      stats.radius = initial_radius_;
    }

    for (size_t i = 0; i < n_threads_; i++) {
      threads.emplace_back([&](){
        DefaultSampler<> sampler((std::random_device()()));
        std::vector<photon_type> photons;

        while (++progress_.current_job <= progress_.total_job) {
          // photon pass
          photons.clear();
          for (size_t j = 0; j < n_photons_; j++) {
            pm.photonTracing(1, std::back_inserter(photons), &sampler);
          }
          auto const photon_map =
            pm.buildPhotonMap(photons.begin(), photons.end());

          // distributed ray tracing pass
          for (size_t y = 0; y < camera.imageHeight(); y++) {
            for (size_t x = 0; x < camera.imageWidth(); x++) {
              auto const hit_point = rayTracing(scene, camera, x, y, sampler);
              if (hit_point.weight.max() == 0) {
                continue;
              }
              auto& stats = statistics.at(x + y * camera.imageWidth());
              updateStatistics(photon_map, hit_point, stats);
            }
          }
        }
      });
    }
    while (!threads.empty()) {
      threads.back().join();
      threads.pop_back();
    }

    image_type image(camera.imageWidth(), camera.imageHeight());
    for (size_t y = 0; y < camera.imageHeight(); y++) {
      for (size_t x = 0; x < camera.imageWidth(); x++) {
        auto& stats = statistics.at(x + y * camera.imageWidth());
        image.at(x, y) =
          stats.flux /
          (kPI * stats.radius * stats.radius) /
          (n_passes_ * n_photons_);
      }
    }

    return image;
  }

private:
  HitPoint rayTracing(
    Scene const& scene,
    camera_type const& camera,
    size_t x,
    size_t y,
    DefaultSampler<>& sampler
  ) const
  {
    HitPoint hit_point;

    ray_type ray = camera.sampleFirstRay(x, y, &sampler);
    hit_type hit;
    Object object;

    for (;;) {
      std::tie(hit, object) = scene.cast(ray);
      if (!hit) {
        hit_point.weight = radiant_type();
        break;
      }

      if (object.surfaceType() == material::SurfaceType::diffuse) {
        hit_point.object = object;
        hit_point.position = hit.position;
        hit_point.normal = hit.normal;
        hit_point.direction = -ray.direction;
        break;
      }

      auto const scatter =
        object.sampleLight(-ray.direction, hit.normal, &sampler);
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, scatter.weight.max());

      if (sampler.uniform<radiant_value_type>() >= p_russian_roulette) {
        hit_point.weight = radiant_type();
        break;
      }

      ray = ray_type(hit.position, scatter.direction);
      hit_point.weight *= scatter.weight / p_russian_roulette;
    }

    return hit_point;
  }

  void updateStatistics(
    photon_map_type const& photon_map,
    HitPoint const& hit_point,
    Statistics& stats
  ) const
  {
    bool expected = false;
    while (stats.locked.compare_exchange_strong(expected, true)) {}

    auto const photons = photon_map.kNearestNeighbours(
      hit_point.position,
      stats.radius,
      k_nearest_photons_
    );

    auto const n = stats.n_photons;
    auto const m = photons.size();
    stats.n_photons = n + m * alpha_;
    if (stats.n_photons == 0) {
      stats.locked = false;
      return;
    }

    stats.radius *= std::sqrt(1. * stats.n_photons / (n + m));

    auto const flux_n = stats.flux;
    auto flux_m =
      hit_point.object.emittance() *
      kPI * stats.radius * stats.radius * n_photons_;
    for (auto const& photon : photons) {
      auto const bsdf = hit_point.object.bsdf(
        photon.direction.template cast<real_type>(),
        hit_point.direction,
        hit_point.normal
      );
      flux_m += bsdf * photon.power;
    }
    flux_m *= hit_point.weight;
    stats.flux = (flux_n + flux_m) * (1. * stats.n_photons / (n + m));

    stats.locked = false;
    return;
  }
};

}
}
