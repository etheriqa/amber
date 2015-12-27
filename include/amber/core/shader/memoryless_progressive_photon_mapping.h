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

#include <iterator>
#include <mutex>
#include <vector>

#include "core/component/kernel.h"
#include "core/component/photon_mapping.h"
#include "core/shader.h"
#include "core/surface_type.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class MemorylessProgressivePhotonMapping : public Shader<Object>
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

  using pm_type = component::PhotonMapping<Object>;

  using photon_map_type = typename pm_type::photon_map_type;
  using photon_type     = typename pm_type::photon_type;

  using kernel_type = component::DiskKernel<real_type>;

  std::double_t initial_radius_, alpha_;

public:
  MemorylessProgressivePhotonMapping(
    std::double_t const initial_radius,
    std::double_t const alpha
  ) noexcept
  : initial_radius_(initial_radius)
  , alpha_(alpha)
  {}

  void Write(std::ostream& os) const noexcept;

  image_type
  operator()(
    scene_type const& scene,
    camera_type const& camera,
    Context& ctx
  );

private:
  void
  Render(
    scene_type const& scene,
    camera_type const& camera,
    kernel_type const& kernel,
    image_type& image_buffer
  ) const;

  static
  photon_map_type
  BuildPhotonMap(
    scene_type const& scene,
    camera_type const& camera,
    Sampler& sampler
  );

  static
  radiant_type const
  RayTracing(
    scene_type const& scene,
    camera_type const& camera,
    photon_map_type const& photon_map,
    std::size_t const u,
    std::size_t const v,
    kernel_type const& kernel,
    Sampler& sampler
  );
};


template <typename Object>
void
MemorylessProgressivePhotonMapping<Object>::Write(
  std::ostream& os
) const noexcept
{
  os
    << "MemorylessProgressivePhotonMapping(initial_radius=" << initial_radius_
    << ", alpha=" << alpha_
    << ")";
}

template <typename Object>
auto
MemorylessProgressivePhotonMapping<Object>::operator()(
  scene_type const& scene,
  camera_type const& camera,
  Context& ctx
)
-> image_type
{
  component::ImageAverageBuffer<radiant_type>
    image(camera.ImageWidth(), camera.ImageHeight());
  {
    std::mutex mtx;
    component::KernelRadiusSequence<real_type> radii(initial_radius_, alpha_);

    IterateParallel(ctx, [&](auto const i){
      image_type image_buffer(camera.ImageWidth(), camera.ImageHeight());
      real_type radius;
      {
        std::lock_guard<std::mutex> lock(mtx);
        radius = radii();
      }

      Render(scene, camera, kernel_type(radius), image_buffer);

      std::lock_guard<std::mutex> lock(mtx);
      image.Buffer(std::move(image_buffer));
    });
  }

  return image;
}

template <typename Object>
void
MemorylessProgressivePhotonMapping<Object>::Render(
  scene_type const& scene,
  camera_type const& camera,
  kernel_type const& kernel,
  image_type& image_buffer
) const
{
  MTSampler sampler((std::random_device()()));

  auto const photon_map = BuildPhotonMap(scene, camera, sampler);

  for (std::size_t v = 0; v < camera.ImageHeight(); v++) {
    for (std::size_t u = 0; u < camera.ImageWidth(); u++) {
      image_buffer.at(u, v) +=
        RayTracing(scene, camera, photon_map, u, v, kernel, sampler);
    }
  }
}

template <typename Object>
auto
MemorylessProgressivePhotonMapping<Object>::BuildPhotonMap(
  scene_type const& scene,
  camera_type const& camera,
  Sampler& sampler
)
-> photon_map_type
{
  pm_type pm(scene);

  std::vector<photon_type> photons;
  {
    photons.reserve(camera.ImageSize());
    for (std::size_t i = 0; i < camera.ImageSize(); i++) {
      pm.PhotonTracing(1, std::back_inserter(photons), sampler);
    }
  }

  return pm.BuildPhotonMap(photons.begin(), photons.end());
}

template <typename Object>
auto
MemorylessProgressivePhotonMapping<Object>::RayTracing(
  scene_type const& scene,
  camera_type const& camera,
  photon_map_type const& photon_map,
  std::size_t const u,
  std::size_t const v,
  kernel_type const& kernel,
  Sampler& sampler
)
-> radiant_type const
{
  ray_type ray;
  radiant_type weight;
  std::tie(ray, weight, std::ignore, std::ignore, std::ignore, std::ignore) =
    camera.GenerateEyeRay(u, v, sampler);

  for (;;) {
    hit_type hit;
    Object object;
    std::tie(hit, object) = scene.Cast(ray);
    if (!hit) {
      break;
    }

    if (object.Surface() == SurfaceType::Light) {
      return weight * object.Radiance(-ray.direction, hit.normal);
    }

    if (object.Surface() == SurfaceType::Diffuse) {
      auto const photons =
        photon_map.SearchRNeighbours(hit.position, kernel.radius());
      radiant_type contribution;
      for (auto const& photon : photons) {
        auto const bsdf =
          object.BSDF(photon.direction, -ray.direction, hit.normal);
        contribution += photon.power * bsdf;
      }
      return contribution * kernel() * weight / camera.ImageSize();
    }

    auto const scatter =
      object.SampleLight(-ray.direction, hit.normal, sampler);
    auto const p_russian_roulette =
      std::min<radiant_value_type>(kRussianRoulette, Max(scatter.weight));

    if (Uniform<radiant_value_type>(sampler) >= p_russian_roulette) {
      break;
    }

    ray.origin = hit.position;
    ray.direction = scatter.direction;
    weight *= scatter.weight / p_russian_roulette;
  }

  return radiant_type();
}

}
}
}
