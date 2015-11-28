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

#include "core/component/kdtree.h"
#include "core/scene.h"

namespace amber {
namespace core {
namespace component {

template <typename Object>
class PhotonMapping
{
private:
  struct Photon;

  using photon_real_type = std::float_t;

public:
  using photon_type     = Photon;
  using photon_map_type = KDTree<Photon, photon_real_type>;

  using scene_type = Scene<Object>;

private:
  using hit_type            = typename Object::hit_type;
  using radiant_type        = typename Object::radiant_type;
  using radiant_value_type  = typename Object::radiant_value_type;
  using ray_type            = typename Object::ray_type;

  using photon_vector3_type = Vector3<photon_real_type>;

  struct Photon
  {
    photon_vector3_type position;
    photon_vector3_type direction;
    radiant_type power;

    operator photon_vector3_type() const noexcept { return position; }
  };

  scene_type scene_;

public:
  explicit PhotonMapping(scene_type const& scene) noexcept : scene_(scene) {}

  template <typename OutputIterator>
  void
  PhotonTracing(
    std::size_t n_photon,
    OutputIterator output,
    Sampler& sampler
  ) const
  {
    ray_type ray;
    radiant_type power;
    std::tie(ray, power, std::ignore, std::ignore, std::ignore, std::ignore) =
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
  photon_map_type
  BuildPhotonMap(
    RandomAccessIterator first,
    RandomAccessIterator last
  ) const
  {
    return photon_map_type(first, last);
  }
};

}
}
}
