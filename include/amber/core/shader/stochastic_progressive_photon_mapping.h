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
#include <vector>

#include "core/component/photon_mapping.h"
#include "core/shader.h"
#include "core/surface_type.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class StochasticProgressivePhotonMapping : public Shader<Object>
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

  struct HitPoint
  {
    Object object;
    vector3_type position;
    unit_vector3_type normal;
    unit_vector3_type direction;
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
    real_type n_photons;
    real_type radius;
    radiant_type flux;
    std::mutex mtx;

    Statistics() noexcept
    : n_photons(),
      radius(),
      flux(),
      mtx()
    {}
  };

  std::double_t initial_radius_, alpha_;

public:
  StochasticProgressivePhotonMapping(
    std::double_t initial_radius,
    std::double_t alpha
  ) noexcept
  : initial_radius_(initial_radius)
  , alpha_(alpha)
  {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "StochasticProgressivePhotonMapping(initial_radius=" << initial_radius_
      << ", alpha=" << alpha_
      << ")";
  }

  image_type
  operator()(
    scene_type const& scene,
    camera_type const& camera,
    Context& ctx
  )
  {
    auto const n_photons = camera.ImageSize();

    std::vector<Statistics> statistics(camera.ImageSize());
    for (auto& stats : statistics) {
      stats.radius = scene.SceneSize() * initial_radius_;
    }

    IterateParallel(ctx, [&](auto const&){
      DefaultSampler<> sampler((std::random_device()()));
      pm_type pm(scene);

      // photon pass
      std::vector<photon_type> photons;
      {
        photons.reserve(n_photons);
        for (std::size_t j = 0; j < n_photons; j++) {
          pm.PhotonTracing(1, std::back_inserter(photons), sampler);
        }
      }
      auto const photon_map =
        pm.BuildPhotonMap(photons.begin(), photons.end());

      // distributed ray tracing pass
      for (std::size_t y = 0; y < camera.ImageHeight(); y++) {
        for (std::size_t x = 0; x < camera.ImageWidth(); x++) {
          auto const hit_point = RayTracing(scene, camera, x, y, sampler);
          if (Max(hit_point.weight) == 0) {
            continue;
          }
          auto& stats = statistics.at(x + y * camera.ImageWidth());
          UpdateStatistics(photon_map, hit_point, n_photons, stats);
        }
      }
    });

    image_type image(camera.ImageWidth(), camera.ImageHeight());
    for (std::size_t y = 0; y < camera.ImageHeight(); y++) {
      for (std::size_t x = 0; x < camera.ImageWidth(); x++) {
        auto& stats = statistics.at(x + y * camera.ImageWidth());
        image.at(x, y) =
          stats.flux /
          (kPI * stats.radius * stats.radius) /
          (ctx.IterationCount() * n_photons);
      }
    }

    return image;
  }

private:
  HitPoint
  RayTracing(
    scene_type const& scene,
    camera_type const& camera,
    std::size_t x,
    std::size_t y,
    DefaultSampler<>& sampler
  ) const
  {
    HitPoint hit_point;

    ray_type ray;
    std::tie(ray, hit_point.weight, std::ignore, std::ignore, std::ignore, std::ignore) =
      camera.GenerateEyeRay(x, y, sampler);

    for (;;) {
      hit_type hit;
      Object object;
      std::tie(hit, object) = scene.Cast(ray);
      if (!hit) {
        hit_point.weight = radiant_type();
        break;
      }

      if (object.Surface() == SurfaceType::Light ||
          object.Surface() == SurfaceType::Diffuse) {
        hit_point.object = object;
        hit_point.position = hit.position;
        hit_point.normal = hit.normal;
        hit_point.direction = -ray.direction;
        break;
      }

      auto const scatter =
        object.SampleLight(-ray.direction, hit.normal, sampler);
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, Max(scatter.weight));

      if (Uniform<radiant_value_type>(sampler) >= p_russian_roulette) {
        hit_point.weight = radiant_type();
        break;
      }

      ray = ray_type(hit.position, scatter.direction);
      hit_point.weight *= scatter.weight / p_russian_roulette;
    }

    return hit_point;
  }

  void
  UpdateStatistics(
    photon_map_type const& photon_map,
    HitPoint const& hit_point,
    std::size_t const n_emitted_photons,
    Statistics& stats
  ) const
  {
    std::lock_guard<std::mutex> lock(stats.mtx);

    auto const photons =
      photon_map.SearchRNeighbours(hit_point.position, stats.radius);

    auto const n_photons = stats.n_photons + photons.size();
    stats.n_photons += photons.size() * alpha_;

    if (n_photons > 0) {
      stats.radius *= std::sqrt(stats.n_photons / n_photons);
    }

    auto flux =
      hit_point.object.Radiance(hit_point.direction, hit_point.normal) *
      kPI * stats.radius * stats.radius * n_emitted_photons;
    for (auto const& photon : photons) {
      auto const bsdf = hit_point.object.BSDF(
        photon.direction,
        hit_point.direction,
        hit_point.normal
      );
      flux += bsdf * photon.power;
    }
    flux *= hit_point.weight;
    stats.flux += flux;
    if (n_photons > 0) {
      stats.flux *= stats.n_photons / n_photons;
    }
  }
};

}
}
}
