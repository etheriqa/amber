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
#include <iterator>
#include <mutex>
#include <thread>
#include <vector>

#include "core/component/photon_mapping.h"
#include "core/shader.h"
#include "core/surface_type.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class PhotonMapping : public Shader<Object>
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

  using pm_type = typename component::PhotonMapping<Object>;

  using photon_map_type = typename pm_type::photon_map_type;
  using photon_type     = typename pm_type::photon_type;

  using vector3_type = Vector3<real_type>;

  std::size_t n_threads_, n_photons_, k_nearest_photons_;
  Progress progress_;

public:
  PhotonMapping(
    std::size_t n_threads,
    std::size_t n_photons,
    std::size_t k_nearest_photons
  ) noexcept
  : n_threads_(n_threads),
    n_photons_(n_photons),
    k_nearest_photons_(k_nearest_photons),
    progress_(3) {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "PhotonMapping(n_threads=" << n_threads_
      << ", n_photons=" << n_photons_
      << ", k_nearest_photons=" << k_nearest_photons_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(scene_type const& scene, camera_type const& camera)
  {
    pm_type pm(scene);
    std::vector<std::thread> threads;
    std::mutex mtx;

    progress_.phase = "Photon Tracing";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = n_photons_;

    std::vector<photon_type> photons;
    for (std::size_t i = 0; i < n_threads_; i++) {
      threads.emplace_back([&](){
        std::vector<photon_type> buffer;
        DefaultSampler<> sampler((std::random_device()()));
        while (progress_.current_job++ < progress_.total_job) {
          pm.PhotonTracing(n_photons_, std::back_inserter(buffer), sampler);
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

    auto const photon_map = pm.BuildPhotonMap(photons.begin(), photons.end());

    progress_.phase = "Distributed Ray Tracing";
    progress_.current_phase = 3;
    progress_.current_job = 0;
    progress_.total_job = camera.imageSize();

    std::vector<std::size_t> pixels(camera.imageSize());
    std::iota(pixels.begin(), pixels.end(), 0);
    std::shuffle(pixels.begin(), pixels.end(), std::random_device());

    image_type image(camera.imageWidth(), camera.imageHeight());
    for (std::size_t i = 0; i < n_threads_; i++) {
      threads.emplace_back([&](){
        DefaultSampler<> sampler((std::random_device()()));
        for (std::size_t j = progress_.current_job++;
             j < progress_.total_job;
             j = progress_.current_job++) {
          auto const x = pixels.at(j) % camera.imageWidth();
          auto const y = pixels.at(j) / camera.imageWidth();
          auto const power = Render(scene, camera, photon_map, x, y, sampler);
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
  Render(
    scene_type const& scene,
    camera_type const& camera,
    photon_map_type const& photon_map,
    std::size_t x, std::size_t y,
    DefaultSampler<>& sampler
  ) const
  {
    ray_type ray;
    radiant_type weight;
    std::tie(ray, weight, std::ignore, std::ignore, std::ignore, std::ignore) =
      camera.GenerateEyeRay(x, y, sampler);

    return DensityEstimate(scene, photon_map, ray, weight, sampler);
  }

  radiant_type
  DensityEstimate(
    scene_type const& scene,
    photon_map_type const& photon_map,
    ray_type const& ray,
    radiant_type const& weight,
    DefaultSampler<>& sampler,
    std::size_t depth = 1
  ) const
  {
    hit_type hit;
    Object object;

    std::tie(hit, object) = scene.Cast(ray);
    if (!hit) {
      return radiant_type();
    }

    if (object.Surface() == SurfaceType::Light) {
      return weight * object.Radiance(-ray.direction, hit.normal);
    }

    if (object.Surface() == SurfaceType::Diffuse) {
      auto const photons =
        photon_map.SearchKNeighbours(hit.position, k_nearest_photons_);
      return weight * Filter(photons, hit, object, -ray.direction);
    }

    auto const p_russian_roulette = RussianRouletteProbability(depth);
    if (Uniform<radiant_value_type>(sampler) >= p_russian_roulette) {
      return radiant_type();
    }

    radiant_type power;

    auto const scatters = object.DistributionLight(-ray.direction, hit.normal);
    for (auto const& scatter : scatters) {
      power += DensityEstimate(
        scene,
        photon_map,
        ray_type(hit.position, scatter.direction),
        weight * scatter.weight / p_russian_roulette,
        sampler,
        depth + 1
      );
    }

    return power;
  }

  radiant_type
  Filter(
    std::vector<photon_type> const& photons,
    hit_type const& hit,
    Object const& object,
    unit_vector3_type const& direction_o
  ) const
  {
    if (photons.empty()) {
      return radiant_type();
    }

    // TODO sum up simply for now
    auto const squared_max_distance =
      SquaredLength(vector3_type(photons.back().position) - hit.position);

    radiant_type power;
    for (auto const& photon : photons) {
      auto const bsdf = object.BSDF(photon.direction, direction_o, hit.normal);
      power += bsdf * photon.power;
    }
    return power / kPI / squared_max_distance;
  }

  radiant_value_type
  RussianRouletteProbability(std::size_t path_length) const noexcept
  {
    return path_length < 8 ? 1 : std::pow(0.5, path_length - 8);
  }
};

}
}
}
