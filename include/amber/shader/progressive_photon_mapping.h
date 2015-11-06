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
class ProgressivePhotonMapping : public Shader<Scene> {
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

  struct HitPoint {
    Object object;
    vector3_type position;
    vector3_type normal;
    vector3_type direction;
    radiant_type weight;
    size_t x;
    size_t y;
    real_type radius;
    size_t n_photon;
    radiant_type flux;
  };

  size_t n_thread_, n_photon_, k_nearest_photon_, n_iteration_;
  double alpha_;
  Progress progress_;

public:
  ProgressivePhotonMapping(size_t n_thread,
                           size_t n_photon,
                           size_t k_nearest_photon,
                           size_t n_iteration,
                           double alpha)
    : n_thread_(n_thread),
      n_photon_(n_photon),
      k_nearest_photon_(k_nearest_photon),
      n_iteration_(n_iteration),
      alpha_(alpha),
      progress_(3) {}

  void write(std::ostream& os) const noexcept {
    os
      << "ProgressivePhotonMapping(n_thread=" << n_thread_
      << ", n_photon=" << n_photon_
      << ", k_nearest_photon=" << k_nearest_photon_
      << ", n_iteration=" << n_iteration_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(Scene const& scene, camera_type const& camera) {
    pm_type pm(scene);
    std::vector<std::thread> threads;
    std::mutex mtx;

    progress_.phase = "Distributed Ray Tracing";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = camera.imageSize();

    std::vector<size_t> pixels(camera.imageSize());
    std::iota(pixels.begin(), pixels.end(), 0);
    std::shuffle(pixels.begin(), pixels.end(), std::random_device());

    std::vector<HitPoint> hit_points;
    for (size_t i = 0; i < n_thread_; i++) {
      threads.emplace_back([&](){
        std::vector<HitPoint> buffer;
        DefaultSampler<> sampler((std::random_device()()));
        for (size_t j = progress_.current_job++;
             j < progress_.total_job;
             j = progress_.current_job++) {
          auto const x = pixels.at(j) % camera.imageWidth();
          auto const y = pixels.at(j) / camera.imageWidth();
          rayTracing(scene, camera, x, y, std::back_inserter(buffer), sampler);
        }
        std::lock_guard<std::mutex> lock(mtx);
        std::move(buffer.begin(), buffer.end(), std::back_inserter(hit_points));
      });
    }
    while (!threads.empty()) {
      threads.back().join();
      threads.pop_back();
    }

    progress_.phase = "Photon Tracing";
    progress_.current_phase = 2;
    progress_.current_job = 0;
    progress_.total_job = n_iteration_;

    for (size_t i = 0; i < n_thread_; i++) {
      threads.emplace_back([&](){
        DefaultSampler<> sampler((std::random_device()()));
        std::vector<photon_type> photons;
        while (++progress_.current_job <= progress_.total_job) {
          photons.clear();
          for (size_t i = 0; i < n_photon_; i++) {
            pm.photonTracing(1, std::back_inserter(photons), &sampler);
          }
          auto const photon_map =
            pm.buildPhotonMap(photons.begin(), photons.end());
          std::lock_guard<std::mutex> lock(mtx);
          progressiveRadianceEstimate(hit_points.begin(),
                                      hit_points.end(),
                                      photon_map);
        }
      });
    }
    while (!threads.empty()) {
      threads.back().join();
      threads.pop_back();
    }

    progress_.phase = "Radiance Evaluation";
    progress_.current_phase = 3;
    progress_.current_job = 0;
    progress_.total_job = 0;

    image_type image(camera.imageWidth(), camera.imageHeight());
    for (auto const& hit_point : hit_points) {
      auto& pixel = image.at(hit_point.x, hit_point.y);

      if (hit_point.object.isEmissive() &&
          dot(hit_point.normal, hit_point.direction) > 0) {
        pixel += hit_point.weight * hit_point.object.emittance();
      }

      pixel +=
        hit_point.weight * hit_point.flux /
        (kPI * hit_point.radius * hit_point.radius) /
        (n_iteration_ * n_photon_);
    }
    return image;
  }

private:
  template <typename OutputIterator>
  void rayTracing(Scene const& scene,
                  camera_type const& camera,
                  size_t x,
                  size_t y,
                  OutputIterator output,
                  DefaultSampler<>& sampler) const {
    auto const ray = camera.sampleFirstRay(x, y, &sampler);
    rayTracing(scene, x, y, ray, radiant_type(1), 0, output, sampler);
  }

  template <typename OutputIterator>
  void rayTracing(Scene const& scene,
                  size_t x,
                  size_t y,
                  ray_type const& ray,
                  radiant_type const& weight,
                  size_t depth,
                  OutputIterator output,
                  DefaultSampler<>& sampler) const {
    hit_type hit;
    Object object;

    std::tie(hit, object) = scene.acceleration()->cast(ray);
    if (!hit) {
      return;
    }

    if (object.surfaceType() == material::SurfaceType::diffuse) {
      HitPoint hit_point;
      hit_point.object    = object;
      hit_point.position  = hit.position;
      hit_point.normal    = hit.normal;
      hit_point.direction = -ray.direction;
      hit_point.weight    = weight;
      hit_point.x         = x;
      hit_point.y         = y;
      hit_point.radius    = 0.01; // TODO
      hit_point.n_photon  = 0;
      hit_point.flux      = radiant_type();
      output = hit_point;
      return;
    }

    if (depth > 10) { // FIXME rewrite with Russian roulette
      return;
    }

    auto const scatters =
      object.distributionImportance(-ray.direction, hit.normal);
    for (auto const& scatter : scatters) {
      auto const new_ray = ray_type(hit.position, scatter.direction);
      auto const new_weight = weight * scatter.weight;
      rayTracing(scene, x, y, new_ray, new_weight, depth + 1, output, sampler);
    }
  }

  template <typename ForwardIterator>
  void progressiveRadianceEstimate(ForwardIterator first,
                                   ForwardIterator last,
                                   photon_map_type const& photon_map) const {
    std::for_each(first, last, [&](auto& hit_point){
      auto const photons =
        photon_map.kNearestNeighbours(hit_point.position,
                                      hit_point.radius,
                                      k_nearest_photon_);
      auto const n = hit_point.n_photon;
      auto const m = photons.size();
      hit_point.n_photon = n + m * alpha_;
      if (hit_point.n_photon == 0) {
        return;
      }

      hit_point.radius *= std::sqrt(1. * hit_point.n_photon / (n + m));

      auto const flux_n = hit_point.flux;
      radiant_type flux_m;
      for (auto const& photon : photons) {
        auto const bsdf =
          hit_point.object.bsdf(photon.direction.template cast<real_type>(),
                                hit_point.direction,
                                hit_point.normal);
        flux_m += bsdf * photon.power;
      }
      hit_point.flux = (flux_n + flux_m) * (1. * hit_point.n_photon / (n + m));
    });
  }
};

}
}
