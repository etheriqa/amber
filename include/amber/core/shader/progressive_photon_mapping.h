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

#include "core/component/photon_mapping.h"
#include "core/shader.h"
#include "core/surface_type.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class ProgressivePhotonMapping : public Shader<Object>
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

  using pm_type            = typename component::PhotonMapping<Object>;

  using photon_map_type    = typename pm_type::photon_map_type;
  using photon_type        = typename pm_type::photon_type;

  struct HitPoint
  {
    Object object;
    vector3_type position;
    vector3_type normal;
    vector3_type direction;
    radiant_type weight;
    size_t x;
    size_t y;
    real_type radius;
    size_t n_photons;
    radiant_type flux;
    std::atomic<bool> locked;

    HitPoint() noexcept {}

    explicit HitPoint(HitPoint const& hp) noexcept
    : object(hp.object),
      position(hp.position),
      normal(hp.normal),
      direction(hp.direction),
      weight(hp.weight),
      x(hp.x),
      y(hp.y),
      radius(hp.radius),
      n_photons(hp.n_photons),
      flux(hp.flux),
      locked(hp.locked.load())
    {}
  };

  size_t n_threads_, n_photons_, k_nearest_photons_, n_iterations_;
  double initial_radius_, alpha_;
  Progress progress_;

public:
  ProgressivePhotonMapping(
    size_t n_threads,
    size_t n_photons,
    size_t k_nearest_photons,
    size_t n_iterations,
    double initial_radius,
    double alpha
  ) noexcept
  : n_threads_(n_threads),
    n_photons_(n_photons),
    k_nearest_photons_(k_nearest_photons),
    n_iterations_(n_iterations),
    initial_radius_(initial_radius),
    alpha_(alpha),
    progress_(3)
  {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "ProgressivePhotonMapping(n_threads=" << n_threads_
      << ", n_photons=" << n_photons_
      << ", k_nearest_photons=" << k_nearest_photons_
      << ", n_iterations=" << n_iterations_
      << ", initial_radius=" << initial_radius_
      << ", alpha=" << alpha_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(scene_type const& scene, camera_type const& camera)
  {
    pm_type pm(scene);
    std::vector<std::thread> threads;

    progress_.phase = "Distributed Ray Tracing";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = camera.imageSize();

    std::vector<HitPoint> hit_points;
    {
      DefaultSampler<> sampler((std::random_device()()));
      for (size_t y = 0; y < camera.imageHeight(); y++) {
        for (size_t x = 0; x < camera.imageWidth(); x++) {
          rayTracing(
            scene,
            camera,
            x,
            y,
            std::back_inserter(hit_points),
            sampler
          );
        }
      }
    }

    progress_.phase = "Photon Tracing";
    progress_.current_phase = 2;
    progress_.current_job = 0;
    progress_.total_job = n_iterations_;

    for (size_t i = 0; i < n_threads_; i++) {
      threads.emplace_back([&](){
        DefaultSampler<> sampler((std::random_device()()));
        std::vector<photon_type> photons;
        while (++progress_.current_job <= progress_.total_job) {
          photons.clear();
          for (size_t i = 0; i < n_photons_; i++) {
            pm.photonTracing(1, std::back_inserter(photons), &sampler);
          }
          progressiveRadianceEstimate(
            hit_points.begin(),
            hit_points.end(),
            pm.buildPhotonMap(photons.begin(), photons.end())
          );
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

      if (Dot(hit_point.normal, hit_point.direction) > 0) {
        pixel += hit_point.weight * hit_point.object.Radiance();
      }

      pixel +=
        hit_point.weight * hit_point.flux /
        (kPI * hit_point.radius * hit_point.radius) /
        (n_iterations_ * n_photons_);
    }
    return image;
  }

private:
  template <typename OutputIterator>
  void rayTracing(
    scene_type const& scene,
    camera_type const& camera,
    size_t x,
    size_t y,
    OutputIterator output,
    DefaultSampler<>& sampler
  ) const
  {
    ray_type ray;
    radiant_type weight;
    std::tie(ray, weight, std::ignore, std::ignore) =
      camera.GenerateRay(x, y, &sampler);

    rayTracing(scene, x, y, ray, weight, 0, output, sampler);
  }

  template <typename OutputIterator>
  void rayTracing(
    scene_type const& scene,
    size_t x,
    size_t y,
    ray_type const& ray,
    radiant_type const& weight,
    size_t depth,
    OutputIterator output,
    DefaultSampler<>& sampler
  ) const
  {
    hit_type hit;
    Object object;

    std::tie(hit, object) = scene.Cast(ray);
    if (!hit) {
      return;
    }

    if (object.Surface() == SurfaceType::Light ||
        object.Surface() == SurfaceType::Diffuse) {
      HitPoint hit_point;
      hit_point.object    = object;
      hit_point.position  = hit.position;
      hit_point.normal    = hit.normal;
      hit_point.direction = -ray.direction;
      hit_point.weight    = weight;
      hit_point.x         = x;
      hit_point.y         = y;
      hit_point.radius    = initial_radius_;
      hit_point.n_photons  = 0;
      hit_point.flux      = radiant_type();
      hit_point.locked    = false;
      output = hit_point;
      return;
    }

    if (depth > 10) { // FIXME rewrite with Russian roulette
      return;
    }

    auto const scatters =
      object.DistributionImportance(-ray.direction, hit.normal);
    for (auto const& scatter : scatters) {
      auto const new_ray = ray_type(hit.position, scatter.direction);
      auto const new_weight = weight * scatter.weight;
      rayTracing(scene, x, y, new_ray, new_weight, depth + 1, output, sampler);
    }
  }

  template <typename ForwardIterator>
  void progressiveRadianceEstimate(
    ForwardIterator first,
    ForwardIterator last,
    photon_map_type const& photon_map
  ) const
  {
    std::for_each(first, last, [&](auto& hit_point){
      bool expected = false;
      while (hit_point.locked.compare_exchange_strong(expected, true)) {}

      auto const photons = photon_map.kNearestNeighbours(
        hit_point.position,
        hit_point.radius,
        k_nearest_photons_
      );

      auto const n = hit_point.n_photons;
      auto const m = photons.size();
      hit_point.n_photons = n + m * alpha_;
      if (hit_point.n_photons == 0) {
        hit_point.locked = false;
        return;
      }

      hit_point.radius *= std::sqrt(1. * hit_point.n_photons / (n + m));

      auto const flux_n = hit_point.flux;
      radiant_type flux_m;
      for (auto const& photon : photons) {
        auto const bsdf = hit_point.object.BSDF(
          photon.direction,
          hit_point.direction,
          hit_point.normal
        );
        flux_m += bsdf * photon.power;
      }
      hit_point.flux = (flux_n + flux_m) * (1. * hit_point.n_photons / (n + m));

      hit_point.locked = false;
    });
  }
};

}
}
}
