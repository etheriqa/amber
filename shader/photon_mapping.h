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

#include "shader/shader.h"

namespace amber {
namespace shader {

template <typename Scene,
          typename Object = typename Scene::object_type>
class PhotonMapping : public Shader<Scene> {
private:
  using camera_type         = typename Shader<Scene>::camera_type;
  using image_type          = typename Shader<Scene>::image_type;

  using hit_type            = typename Object::hit_type;
  using radiant_type        = typename Object::radiant_type;
  using radiant_value_type  = typename Object::radiant_value_type;
  using ray_type            = typename Object::ray_type;
  using real_type           = typename Object::real_type;
  using vector3_type        = typename Object::vector3_type;

  using photon_real_type    = std::float_t;
  using photon_aabb_type    = geometry::AABB<photon_real_type>;
  using photon_vector3_type = geometry::Vector3<photon_real_type>;

  enum struct Axis {
    x,
    y,
    z,
  };

  struct Photon {
    photon_vector3_type position;
    photon_vector3_type direction;
    radiant_type power;
    Axis axis;
  };

  struct PhotonComparator {
    photon_vector3_type point;

    explicit PhotonComparator(vector3_type const& point) noexcept
      : point(point.template cast<photon_real_type>()) {}

    explicit PhotonComparator(photon_vector3_type const& point) noexcept
      : point(point) {}

    bool operator()(Photon const& a, Photon const& b) const noexcept {
      return
        (a.position - point).squaredLength()
        < (b.position - point).squaredLength();
    }
  };

  struct PhotonMap {
    std::vector<Photon> photons_;

    explicit PhotonMap(std::vector<Photon> photons) noexcept
      : photons_(photons.size()) {
      buildPhotonMap(photons.begin(), photons.end(), 0);
    }

    template <typename RandomAccessIterator>
    void buildPhotonMap(RandomAccessIterator first,
                        RandomAccessIterator last,
                        size_t pos) {
      if (pos >= photons_.size()) {
        return;
      }
      photon_aabb_type aabb;
      std::for_each(first, last, [&](auto const& photon){
        aabb += photon_aabb_type(photon.position);
      });
      auto const x = aabb.max.x() - aabb.min.x();
      auto const y = aabb.max.y() - aabb.min.y();
      auto const z = aabb.max.z() - aabb.min.z();
      Axis axis;
      if (x > y && x > z) {
        axis = Axis::x;
      } else if (y > z) {
        axis = Axis::y;
      } else {
        axis = Axis::z;
      }
      std::sort(first, last, [&](auto const& a, auto const& b){
          switch (axis) {
          case Axis::x:
            return a.position.x() < b.position.x();
          case Axis::y:
            return a.position.y() < b.position.y();
          case Axis::z:
            return a.position.z() < b.position.z();
          }
      });
      const size_t size = std::distance(first, last);
      size_t left_size = 0, right_size = 0;
      while (left_size + right_size + 1 < size) {
        left_size = std::min(size - 1 - right_size, (left_size << 1) + 1);
        right_size = std::min(size - 1 - left_size, (right_size << 1) + 1);
      }
      auto const middle = first + left_size;
      photons_[pos] = *middle;
      photons_[pos].axis = axis;
      buildPhotonMap(first, middle, pos * 2 + 1);
      buildPhotonMap(middle + 1, last, pos * 2 + 2);
    }

    std::vector<Photon> kNearestNeighbours(vector3_type const& point,
                                           real_type radius,
                                           size_t k) const {
      std::vector<Photon> heap;
      heap.reserve(k + 1);
      kNearestNeighbours(heap,
                         0,
                         point.template cast<photon_real_type>(),
                         radius * radius,
                         k);
      std::sort_heap(heap.begin(), heap.end(), PhotonComparator(point));
      return heap;
    }

    void kNearestNeighbours(std::vector<Photon>& heap,
                            size_t pos,
                            photon_vector3_type const& point,
                            photon_real_type squared_radius,
                            size_t k) const {
      if (pos >= photons_.size()) {
        return;
      }
      auto const& photon = photons_[pos];
      real_type plane_distance;
      switch (photon.axis) {
      case Axis::x:
        plane_distance = point.x() - photon.position.x();
        break;
      case Axis::y:
        plane_distance = point.y() - photon.position.y();
        break;
      case Axis::z:
        plane_distance = point.z() - photon.position.z();
        break;
      }
      size_t near, far;
      if (plane_distance < 0) {
        near = pos * 2 + 1;
        far = pos * 2 + 2;
      } else {
        near = pos * 2 + 2;
        far = pos * 2 + 1;
      }
      kNearestNeighbours(heap, near, point, squared_radius, k);
      if (heap.size() >= k) {
        squared_radius = (heap.front().position - point).squaredLength();
      }
      if (plane_distance * plane_distance < squared_radius) {
        kNearestNeighbours(heap, far, point, squared_radius, k);
      }
      if ((photon.position - point).squaredLength() < squared_radius) {
        heap.push_back(photon);
        std::push_heap(heap.begin(), heap.end(), PhotonComparator(point));
      }
      while (heap.size() > k) {
        std::pop_heap(heap.begin(), heap.end(), PhotonComparator(point));
        heap.pop_back();
      }
    }
  };

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
    std::vector<std::thread> threads;
    std::mutex mtx;

    progress_.phase = "Photon Tracing";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = n_photon_;

    std::vector<Photon> photons;
    for (size_t i = 0; i < n_thread_; i++) {
      threads.emplace_back([&](){
        std::vector<Photon> buffer;
        DefaultSampler<> sampler((std::random_device()()));
        while (progress_.current_job++ < progress_.total_job) {
          photonTracing(scene, std::back_inserter(buffer), sampler);
        }
        std::lock_guard<std::mutex> lock(mtx);
        std::move(buffer.begin(), buffer.end(), std::back_inserter(photons));
      });
    }
    while (!threads.empty()) {
      threads.back().join();
      threads.pop_back();
    }

    progress_.phase = "Construct Photon Maps";
    progress_.current_phase = 2;
    progress_.current_job = 0;
    progress_.total_job = 0;

    PhotonMap const photon_map(std::move(photons));

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
  template <typename OutputIterator>
  void photonTracing(Scene const& scene,
                     OutputIterator output,
                     DefaultSampler<>& sampler) const {
    auto const light = (*scene.light_sampler())(&sampler);
    auto power = scene.light_sampler()->total_power() / n_photon_;
    ray_type ray = light.sampleFirstRay(&sampler);
    hit_type hit;
    Object object;

    for (;;) {
      std::tie(hit, object) = scene.acceleration()->cast(ray);
      if (!hit) {
        break;
      }

      if (object.surfaceType() == material::SurfaceType::diffuse) {
        Photon photon;
        photon.position = hit.position.template cast<photon_real_type>();
        photon.direction = -ray.direction.template cast<photon_real_type>();
        photon.power = power;
        output = photon;
      }

      auto const sample =
        object.sampleScatter(-ray.direction, hit.normal, &sampler);
      ray = ray_type(hit.position, sample.direction_o);
      auto const reflectance = sample.bsdf / sample.psa_probability;
      power *= reflectance;

      if (object.surfaceType() == material::SurfaceType::specular) {
        continue;
      }

      auto const p_russian_roulette = reflectance.max();
      if (sampler.uniform<real_type>() >= p_russian_roulette) {
        break;
      }
      power /= std::min<real_type>(1, p_russian_roulette);
    }
  }

  radiant_type
  rendering(Scene const& scene,
            camera_type const& camera,
            PhotonMap const& photon_map,
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
                PhotonMap const& photon_map,
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
        photon_map.kNearestNeighbours(hit.position, 1, k_nearest_photon_);
      return
        power + weight * gaussianFilter(photons, hit, object, -ray.direction);
    }

    if (depth > 10) { // FIXME inconsistently; need to rewrite with Russian roulette
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
  gaussianFilter(std::vector<Photon> const& photons,
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
