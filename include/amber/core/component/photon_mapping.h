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

#include <mutex>
#include <numeric>

#include "core/aabb.h"
#include "core/axis.h"
#include "core/scene.h"
#include "core/vector3.h"

namespace amber {
namespace core {
namespace component {

template <typename Object>
class PhotonMapping
{
private:
  struct Photon;
  struct PhotonMap;

public:
  using photon_type         = Photon;
  using photon_map_type     = PhotonMap;

  using scene_type          = Scene<Object>;

private:
  using hit_type            = typename Object::hit_type;
  using radiant_type        = typename Object::radiant_type;
  using radiant_value_type  = typename Object::radiant_value_type;
  using ray_type            = typename Object::ray_type;

  using photon_real_type    = std::float_t;
  using photon_vector3_type = Vector3<photon_real_type>;
  using photon_aabb_type    = AABB<photon_real_type>;

  struct Photon
  {
    photon_vector3_type position;
    photon_vector3_type direction;
    radiant_type power;
    Axis axis;
  };

  struct PhotonComparator
  {
    photon_vector3_type point;

    explicit PhotonComparator(photon_vector3_type const& point) noexcept
    : point(point)
    {}

    bool operator()(Photon const& a, Photon const& b) const noexcept
    {
      return
        SquaredLength(a.position - point) < SquaredLength(b.position - point);
    }
  };

  struct PhotonMap
  {
    std::vector<Photon> photons_;

    std::mutex mutable search_buffer_mtx_;
    std::vector<Photon> mutable search_buffer_;

    template <typename RandomAccessIterator>
    PhotonMap(RandomAccessIterator first, RandomAccessIterator last)
    : photons_(std::distance(first, last))
    {
      AllocatePhotons(first, last, 0);
    }

    PhotonMap(PhotonMap const& photon_map) : photons_(photon_map.photons_) {}

    template <typename RandomAccessIterator>
    void AllocatePhotons(
      RandomAccessIterator first,
      RandomAccessIterator last,
      size_t pos
    )
    {
      if (pos >= photons_.size()) {
        return;
      }
      photon_aabb_type aabb;
      std::for_each(first, last, [&](auto const& photon){
        aabb += photon_aabb_type(photon.position);
      });
      auto const x = aabb.max().x() - aabb.min().x();
      auto const y = aabb.max().y() - aabb.min().y();
      auto const z = aabb.max().z() - aabb.min().z();
      Axis axis;
      if (x > y && x > z) {
        axis = Axis::X;
      } else if (y > z) {
        axis = Axis::Y;
      } else {
        axis = Axis::Z;
      }
      std::sort(first, last, [&](auto const& a, auto const& b){
        switch (axis) {
        case Axis::X:
          return a.position.x() < b.position.x();
        case Axis::Y:
          return a.position.y() < b.position.y();
        case Axis::Z:
          return a.position.z() < b.position.z();
        }
      });
      size_t const size = std::distance(first, last);
      size_t left_size = 0, right_size = 0;
      while (left_size + right_size + 1 < size) {
        left_size = std::min(size - 1 - right_size, (left_size << 1) + 1);
        right_size = std::min(size - 1 - left_size, (right_size << 1) + 1);
      }
      auto const middle = first + left_size;
      photons_[pos] = *middle;
      photons_[pos].axis = axis;
      AllocatePhotons(first, middle, pos * 2 + 1);
      AllocatePhotons(middle + 1, last, pos * 2 + 2);
    }

    std::vector<Photon>
    KNeighbours(
      photon_vector3_type const& point,
      size_t k
    ) const
    {
      return SearchNeighbours(
        point,
        k,
        std::numeric_limits<photon_real_type>::max()
      );
    }

    std::vector<Photon>
    RNeighbours(
      photon_vector3_type const& point,
      photon_real_type radius
    ) const
    {
      return SearchNeighbours(point, photons_.size(), radius * radius);
    }

    std::vector<Photon>
    SearchNeighbours(
      photon_vector3_type const& point,
      size_t k,
      photon_real_type squared_radius
    ) const
    {
      std::lock_guard<std::mutex> lock(search_buffer_mtx_);
      search_buffer_.clear();
      SearchNeighbours(0, point, k, squared_radius);
      std::sort_heap(
        search_buffer_.begin(),
        search_buffer_.end(),
        PhotonComparator(point)
      );
      return search_buffer_;
    }

    void SearchNeighbours(
      size_t pos,
      photon_vector3_type const& point,
      size_t k,
      photon_real_type& squared_radius
    ) const
    {
      if (pos >= photons_.size()) {
        return;
      }
      auto const& photon = photons_[pos];
      photon_real_type plane_distance;
      switch (photon.axis) {
      case Axis::X:
        plane_distance = point.x() - photon.position.x();
        break;
      case Axis::Y:
        plane_distance = point.y() - photon.position.y();
        break;
      case Axis::Z:
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
      SearchNeighbours(near, point, k, squared_radius);
      if (SquaredLength(photon.position - point) < squared_radius) {
        search_buffer_.push_back(photon);
        std::push_heap(
          search_buffer_.begin(),
          search_buffer_.end(),
          PhotonComparator(point)
        );
        while (search_buffer_.size() > k) {
          std::pop_heap(
            search_buffer_.begin(),
            search_buffer_.end(),
            PhotonComparator(point)
          );
          search_buffer_.pop_back();
        }
        if (search_buffer_.size() >= k) {
          squared_radius =
            SquaredLength(search_buffer_.front().position - point);
        }
      }
      if (plane_distance * plane_distance < squared_radius) {
        SearchNeighbours(far, point, k, squared_radius);
      }
    }
  };

  scene_type scene_;

public:
  explicit PhotonMapping(scene_type const& scene) noexcept : scene_(scene) {}

  template <typename OutputIterator>
  void PhotonTracing(
    size_t n_photon,
    OutputIterator output,
    Sampler& sampler
  ) const
  {
    ray_type ray;
    radiant_type power;
    std::tie(ray, power, std::ignore, std::ignore, std::ignore) =
      scene_.GenerateLightRay(sampler);
    power /= n_photon;

    for (;;) {
      hit_type hit;
      Object object;
      std::tie(hit, object) = scene_.Cast(ray);
      if (!hit) {
        break;
      }

      if (object.Surface() == SurfaceType::Light ||
          object.Surface() == SurfaceType::Diffuse) {
        Photon photon;
        photon.position = hit.position;
        photon.direction = -ray.direction;
        photon.power = power;
        output = photon;
      }

      auto const scatter =
        object.SampleImportance(-ray.direction, hit.normal, sampler);
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, Max(scatter.weight));

      if (Uniform<radiant_value_type>(sampler) >= p_russian_roulette) {
        break;
      }

      ray = ray_type(hit.position, scatter.direction);
      power *= scatter.weight / p_russian_roulette;
    }
  }

  template <typename RandomAccessIterator>
  PhotonMap BuildPhotonMap(
    RandomAccessIterator first,
    RandomAccessIterator last
  ) const
  {
    return PhotonMap(first, last);
  }
};

}
}
}
