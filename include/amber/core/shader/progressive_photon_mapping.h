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
class ProgressivePhotonMapping : public Shader<Object>
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
    std::size_t x;
    std::size_t y;
    real_type n_photons;
    real_type radius;
    radiant_type flux;
    std::mutex mtx;

    HitPoint() noexcept {}

    explicit HitPoint(HitPoint const& hp) noexcept
    : object(hp.object),
      position(hp.position),
      normal(hp.normal),
      direction(hp.direction),
      weight(hp.weight),
      x(hp.x),
      y(hp.y),
      n_photons(hp.n_photons),
      radius(hp.radius),
      flux(hp.flux),
      mtx()
    {}
  };

  std::double_t initial_radius_, alpha_;

public:
  ProgressivePhotonMapping(
    std::double_t initial_radius,
    std::double_t alpha
  ) noexcept
  : initial_radius_(initial_radius)
  , alpha_(alpha)
  {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "ProgressivePhotonMapping(initial_radius=" << initial_radius_
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
    auto const n_photons = camera.imageSize();

    // distributed ray tracing
    std::vector<HitPoint> hit_points;
    {
      DefaultSampler<> sampler((std::random_device()()));
      for (std::size_t y = 0; y < camera.imageHeight(); y++) {
        for (std::size_t x = 0; x < camera.imageWidth(); x++) {
          RayTracing(
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

    // photon tracing
    IterateParallel(ctx, [&](auto const&){
      DefaultSampler<> sampler((std::random_device()()));
      pm_type pm(scene);

      std::vector<photon_type> photons;
      {
        photons.reserve(n_photons);
        for (std::size_t i = 0; i < n_photons; i++) {
          pm.PhotonTracing(1, std::back_inserter(photons), sampler);
        }
      }

      ProgressiveDensityEstimate(
        hit_points.begin(),
        hit_points.end(),
        pm.BuildPhotonMap(photons.begin(), photons.end())
      );
    });

    // measurement evaluation
    image_type image(camera.imageWidth(), camera.imageHeight());
    for (auto const& hit_point : hit_points) {
      auto& pixel = image.at(hit_point.x, hit_point.y);

      pixel +=
        hit_point.weight *
        hit_point.object.Radiance(hit_point.direction, hit_point.normal);

      pixel +=
        hit_point.weight * hit_point.flux /
        (kPI * hit_point.radius * hit_point.radius) /
        (ctx.IterationCount() * n_photons);
    }
    return image;
  }

private:
  template <typename OutputIterator>
  void
  RayTracing(
    scene_type const& scene,
    camera_type const& camera,
    std::size_t x,
    std::size_t y,
    OutputIterator output,
    DefaultSampler<>& sampler
  ) const
  {
    ray_type ray;
    radiant_type weight;
    std::tie(ray, weight, std::ignore, std::ignore, std::ignore, std::ignore) =
      camera.GenerateEyeRay(x, y, sampler);

    RayTracing(scene, x, y, ray, weight, 1, output, sampler);
  }

  template <typename OutputIterator>
  void
  RayTracing(
    scene_type const& scene,
    std::size_t x,
    std::size_t y,
    ray_type const& ray,
    radiant_type const& weight,
    std::size_t depth,
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
      hit_point.n_photons = 0;
      hit_point.radius    = scene.SceneSize() * initial_radius_;
      hit_point.flux      = radiant_type();
      output = hit_point;
      return;
    }

    auto const p_russian_roulette = RussianRouletteProbability(depth);
    if (Uniform<radiant_value_type>(sampler) >= p_russian_roulette) {
      return;
    }

    auto const scatters =
      object.DistributionImportance(-ray.direction, hit.normal);
    for (auto const& scatter : scatters) {
      auto const new_ray = ray_type(hit.position, scatter.direction);
      auto const new_weight = weight * scatter.weight / p_russian_roulette;
      RayTracing(scene, x, y, new_ray, new_weight, depth + 1, output, sampler);
    }
  }

  template <typename ForwardIterator>
  void
  ProgressiveDensityEstimate(
    ForwardIterator first,
    ForwardIterator last,
    photon_map_type const& photon_map
  ) const
  {
    std::for_each(first, last, [&](auto& hit_point){
      std::lock_guard<std::mutex> lock(hit_point.mtx);

      auto const photons =
        photon_map.SearchRNeighbours(hit_point.position, hit_point.radius);

      auto const n_photons = hit_point.n_photons + photons.size();
      hit_point.n_photons += photons.size() * alpha_;

      if (n_photons > 0) {
        hit_point.radius *= std::sqrt(hit_point.n_photons / n_photons);
      }

      radiant_type flux;
      for (auto const& photon : photons) {
        auto const bsdf = hit_point.object.BSDF(
          photon.direction,
          hit_point.direction,
          hit_point.normal
        );
        flux += bsdf * photon.power;
      }
      hit_point.flux += flux;
      if (n_photons > 0) {
        hit_point.flux *= hit_point.n_photons / n_photons;
      }
    });
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
