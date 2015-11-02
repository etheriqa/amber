/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/axis.h"
#include "geometry/aabb.h"
#include "geometry/vector.h"

namespace amber {
namespace shader {
namespace framework {

template <typename Scene,
          typename Object = typename Scene::object_type>
class PhotonMapping {
private:
  struct Photon;
  struct PhotonMap;

public:
  using photon_type         = Photon;
  using photon_map_type     = PhotonMap;

private:
  using hit_type            = typename Object::hit_type;
  using radiant_type        = typename Object::radiant_type;
  using radiant_value_type  = typename Object::radiant_value_type;
  using ray_type            = typename Object::ray_type;

  using photon_real_type    = std::float_t;
  using photon_vector3_type = geometry::Vector3<photon_real_type>;
  using photon_aabb_type    = geometry::AABB<photon_real_type>;

  struct Photon {
    photon_vector3_type position;
    photon_vector3_type direction;
    radiant_type power;
    Axis axis;
  };

  struct PhotonComparator {
    photon_vector3_type point;

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

    template <typename RandomAccessIterator>
    PhotonMap(RandomAccessIterator first, RandomAccessIterator last)
      : photons_(std::distance(first, last)) {
      allocatePhotons(first, last, 0);
    }

    template <typename RandomAccessIterator>
    void allocatePhotons(RandomAccessIterator first,
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
      allocatePhotons(first, middle, pos * 2 + 1);
      allocatePhotons(middle + 1, last, pos * 2 + 2);
    }

    template <typename T>
    std::vector<Photon> kNearestNeighbours(geometry::Vector3<T> const& point,
                                           T radius,
                                           size_t k) const {
      std::vector<Photon> heap;
      heap.reserve(k + 1);
      kNearestNeighbours(heap,
                         0,
                         point.template cast<photon_real_type>(),
                         radius * radius,
                         k);
      std::sort_heap(heap.begin(), heap.end(),
                     PhotonComparator(point.template cast<photon_real_type>()));
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
      photon_real_type plane_distance;
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

  Scene scene_;

public:
  explicit PhotonMapping(Scene const& scene) noexcept
    : scene_(scene) {}

  template <typename OutputIterator>
  void photonTracing(size_t n_photon,
                     OutputIterator output,
                     Sampler* sampler) const {
    auto const light = (*scene_.light_sampler())(sampler);
    auto power = scene_.light_sampler()->total_power() / n_photon;
    ray_type ray = light.sampleFirstRay(sampler);
    hit_type hit;
    Object object;

    for (;;) {
      std::tie(hit, object) = scene_.acceleration()->cast(ray);
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
        object.sampleLightScatter(-ray.direction, hit.normal, sampler);
      ray = ray_type(hit.position, sample.direction_o);
      auto const reflectance = sample.bsdf / sample.psa_probability;
      power *= reflectance;

      if (object.surfaceType() == material::SurfaceType::specular) {
        continue;
      }

      auto const p_russian_roulette = reflectance.max();
      if (sampler->uniform<radiant_value_type>() >= p_russian_roulette) {
        break;
      }
      power /= std::min<radiant_value_type>(1, p_russian_roulette);
    }
  }

  template <typename RandomAccessIterator>
  PhotonMap buildPhotonMap(RandomAccessIterator first,
                           RandomAccessIterator last) const {
    return PhotonMap(first, last);
  }
};

}
}
}
