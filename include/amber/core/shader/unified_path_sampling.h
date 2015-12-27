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

#include <limits>
#include <memory>
#include <mutex>
#include <vector>

#include "core/component/bidirectional_path_tracing.h"
#include "core/component/image_average_buffer.h"
#include "core/component/kdtree.h"
#include "core/component/kernel.h"
#include "core/component/multiple_importance_sampling.h"
#include "core/shader.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class UnifiedPathSampling : public Shader<Object>
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

  using event_type       = component::SubpathEvent<radiant_type, real_type>;
  using path_buffer_type = component::PathBuffer<radiant_type, real_type>;

  using path_type = std::vector<event_type>;

  using vector3_type = Vector3<real_type>;

  struct Photon
  {
    std::size_t path_index;
    std::size_t event_index;
    vector3_type position;

    Photon() noexcept
    : path_index(std::numeric_limits<std::size_t>::max())
    , event_index(std::numeric_limits<std::size_t>::max())
    , position()
    {}

    Photon(
      std::size_t const path_index,
      std::size_t const event_index,
      vector3_type const& position
    ) noexcept
    : path_index(path_index)
    , event_index(event_index)
    , position(position)
    {}
  };

  using photon_map_type = component::KDTree<Photon, real_type>;

  using kernel_type = component::DiskKernel<real_type>;

  std::double_t initial_radius_, alpha_;

public:
  UnifiedPathSampling(
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
    path_buffer_type& path_buffer,
    image_type& image_buffer
  ) const;

  static
  std::vector<path_type>
  GenerateLightPaths(
    scene_type const& scene,
    camera_type const& camera,
    Sampler& sampler
  );

  static
  std::vector<path_type>
  GenerateEyePaths(
    scene_type const& scene,
    camera_type const& camera,
    Sampler& sampler
  );

  static
  photon_map_type
  BuildPhotonMap(std::vector<path_type> const& light_paths);

  static
  radiant_type const
  CombinePath(
    scene_type const& scene,
    camera_type const& camera,
    photon_map_type const& photon_map,
    std::vector<path_type> const& light_paths,
    path_type const& light_path,
    path_type const& eye_path,
    kernel_type const& kernel,
    path_buffer_type& path_buffer,
    image_type& image_buffer
  );

  static
  radiant_type const
  ConnectEyeImage(
    scene_type const& scene,
    camera_type const& camera,
    path_type const& light_path,
    path_type const& eye_path,
    kernel_type const& kernel,
    path_buffer_type& path_buffer
  );

  static
  void
  ConnectLightImage(
    scene_type const& scene,
    camera_type const& camera,
    path_type const& light_path,
    path_type const& eye_path,
    kernel_type const& kernel,
    path_buffer_type& path_buffer,
    image_type& image_buffer
  );

  static
  radiant_type const
  ConnectDensityEstimation(
    scene_type const& scene,
    camera_type const& camera,
    photon_map_type const& photon_map,
    std::vector<path_type> const& light_paths,
    path_type const& eye_path,
    kernel_type const& kernel,
    path_buffer_type& path_buffer
  );

  template <bool VirtualPerturbation>
  static
  radiant_value_type const
  MISWeight(
    scene_type const& scene,
    camera_type const& camera,
    path_type const& light_path,
    path_type const& eye_path,
    std::size_t const s,
    std::size_t const t,
    kernel_type const& kernel,
    path_buffer_type& path_buffer
  );
};

template <typename Object>
void
UnifiedPathSampling<Object>::Write(std::ostream& os) const noexcept
{
  os
    << "UnifiedPathSampling(initial_radius=" << initial_radius_
    << ", alpha=" << alpha_
    << ")";
}

template <typename Object>
auto
UnifiedPathSampling<Object>::operator()(
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
      path_buffer_type path_buffer;
      image_type image_buffer(camera.ImageWidth(), camera.ImageHeight());
      real_type radius;
      {
        std::lock_guard<std::mutex> lock(mtx);
        radius = radii();
      }

      Render(scene, camera, kernel_type(radius), path_buffer, image_buffer);

      std::lock_guard<std::mutex> lock(mtx);
      image.Buffer(std::move(image_buffer));
    });
  }

  return image;
}

template <typename Object>
void
UnifiedPathSampling<Object>::Render(
  scene_type const& scene,
  camera_type const& camera,
  kernel_type const& kernel,
  path_buffer_type& path_buffer,
  image_type& image_buffer
) const
{
  MTSampler sampler((std::random_device()()));

  auto const light_paths = GenerateLightPaths(scene, camera, sampler);
  auto const eye_paths = GenerateEyePaths(scene, camera, sampler);
  auto const photon_map = BuildPhotonMap(light_paths);

  for (std::size_t i = 0; i < camera.ImageSize(); i++) {
    auto const& light_path = light_paths.at(i);
    auto const& eye_path = eye_paths.at(i);

    std::size_t u, v;
    std::tie(u, v) = camera.UnpackUV(i);

    image_buffer.at(u, v) += CombinePath(
      scene,
      camera,
      photon_map,
      light_paths,
      light_path,
      eye_path,
      kernel,
      path_buffer,
      image_buffer
    );
  }
}

template <typename Object>
auto
UnifiedPathSampling<Object>::GenerateLightPaths(
  scene_type const& scene,
  camera_type const& camera,
  Sampler& sampler
)
-> std::vector<path_type>
{
  std::vector<path_type> paths;
  paths.reserve(camera.ImageSize());

  for (std::size_t i = 0; i < camera.ImageSize(); i++) {
    paths.emplace_back(
      component::GenerateLightSubpath(scene, camera, sampler)
    );
  }

  return paths;
}

template <typename Object>
auto
UnifiedPathSampling<Object>::GenerateEyePaths(
  scene_type const& scene,
  camera_type const& camera,
  Sampler& sampler
)
-> std::vector<path_type>
{
  std::vector<path_type> paths;
  paths.reserve(camera.ImageSize());

  for (std::size_t i = 0; i < camera.ImageSize(); i++) {
    std::size_t u, v;
    std::tie(u, v) = camera.UnpackUV(i);
    paths.emplace_back(
      component::GenerateEyeSubpath(scene, camera, u, v, sampler)
    );
  }

  return paths;
}

template <typename Object>
auto
UnifiedPathSampling<Object>::BuildPhotonMap(
  std::vector<path_type> const& light_paths
)
-> photon_map_type
{
  std::vector<Photon> photons;

  for (std::size_t i = 0; i < light_paths.size(); i++) {
    auto const& light_path = light_paths.at(i);
    for (std::size_t j = 0; j < light_path.size(); j++) {
      auto const& event = light_path.at(j);
      if (event.object.Surface() == SurfaceType::Diffuse) {
        photons.emplace_back(i, j, event.position);
      }
    }
  }

  return photon_map_type(
    photons.begin(),
    photons.end(),
    [](auto const& photon){ return photon.position; }
  );
}

template <typename Object>
auto
UnifiedPathSampling<Object>::CombinePath(
  scene_type const& scene,
  camera_type const& camera,
  photon_map_type const& photon_map,
  std::vector<path_type> const& light_paths,
  path_type const& light_path,
  path_type const& eye_path,
  kernel_type const& kernel,
  path_buffer_type& path_buffer,
  image_type& image_buffer
)
-> radiant_type const
{
  auto const contribution_eye = ConnectEyeImage(
    scene,
    camera,
    light_path,
    eye_path,
    kernel,
    path_buffer
  );

  ConnectLightImage(
    scene,
    camera,
    light_path,
    eye_path,
    kernel,
    path_buffer,
    image_buffer
  );

  auto const contribution_de = ConnectDensityEstimation(
    scene,
    camera,
    photon_map,
    light_paths,
    eye_path,
    kernel,
    path_buffer
  );

  return contribution_eye + contribution_de / camera.ImageSize();
}

template <typename Object>
auto
UnifiedPathSampling<Object>::ConnectEyeImage(
  scene_type const& scene,
  camera_type const& camera,
  path_type const& light_path,
  path_type const& eye_path,
  kernel_type const& kernel,
  path_buffer_type& path_buffer
)
-> radiant_type const
{
  radiant_type measurement;

  for (std::size_t s = 0; s <= light_path.size(); s++) {
    for (std::size_t t = 2; t <= eye_path.size(); t++) {
      auto contribution = component::Connect(
        scene,
        camera,
        s == 0 ? nullptr : &light_path.at(s - 1),
        t == 0 ? nullptr : &eye_path.at(t - 1)
      );

      if (!contribution) {
        continue;
      }

      auto const weight = MISWeight<true>(
        scene,
        camera,
        light_path,
        eye_path,
        s,
        t,
        kernel,
        path_buffer
      );

      measurement += contribution.measurement * weight;
    }
  }

  return measurement;
}

template <typename Object>
void
UnifiedPathSampling<Object>::ConnectLightImage(
  scene_type const& scene,
  camera_type const& camera,
  path_type const& light_path,
  path_type const& eye_path,
  kernel_type const& kernel,
  path_buffer_type& path_buffer,
  image_type& image_buffer
)
{
  for (std::size_t s = 1; s <= light_path.size(); s++) {
    for (std::size_t t = 0; t < 2; t++) {
      auto contribution = component::Connect(
        scene,
        camera,
        s == 0 ? nullptr : &light_path.at(s - 1),
        t == 0 ? nullptr : &eye_path.at(t - 1)
      );

      if (!contribution) {
        continue;
      }

      auto const& u = std::get<0>(*contribution.pixel);
      auto const& v = std::get<1>(*contribution.pixel);

      auto const weight = MISWeight<true>(
        scene,
        camera,
        light_path,
        eye_path,
        s,
        t,
        kernel,
        path_buffer
      );

      image_buffer.at(u, v) +=
        contribution.measurement * weight / camera.ImageSize();
    }
  }
}

template <typename Object>
auto
UnifiedPathSampling<Object>::ConnectDensityEstimation(
  scene_type const& scene,
  camera_type const& camera,
  photon_map_type const& photon_map,
  std::vector<path_type> const& light_paths,
  path_type const& eye_path,
  kernel_type const& kernel,
  path_buffer_type& path_buffer
)
-> radiant_type const
{
  radiant_type measurement;

  for (std::size_t t = 2; t <= eye_path.size(); t++) {
    auto const& eye_end = eye_path.at(t - 1);

    if (eye_end.object.Surface() != SurfaceType::Diffuse) {
      continue;
    }

    auto const photons =
      photon_map.SearchRNeighbours(eye_end.position, kernel.radius());

    radiant_type contribution;

    for (auto const& photon : photons) {
      auto const s = photon.event_index + 1;
      auto const& light_path = light_paths.at(photon.path_index);
      auto const& light_end = light_path.at(photon.event_index);

      if (light_end.object != eye_end.object) {
        continue;
      }

      auto const bsdf = eye_end.object.BSDF(
        light_end.direction,
        eye_end.direction,
        eye_end.normal
      );

      auto const weight = MISWeight<false>(
        scene,
        camera,
        light_path,
        eye_path,
        s,
        t,
        kernel,
        path_buffer
      );

      contribution += light_end.weight * bsdf * weight;
    }

    measurement += contribution * kernel() * eye_end.weight;
  }

  return measurement;
}

template <typename Object>
template <bool VirtualPerturbation>
auto
UnifiedPathSampling<Object>::MISWeight(
  scene_type const& scene,
  camera_type const& camera,
  path_type const& light_path,
  path_type const& eye_path,
  std::size_t const s,
  std::size_t const t,
  kernel_type const& kernel,
  path_buffer_type& path_buffer
)
-> radiant_value_type const
{
  auto const s_ups = s + VirtualPerturbation;
  auto const t_ups = t;

  path_buffer.Buffer(scene, camera, light_path, eye_path, s_ups - 1, t_ups);

  auto const k = s_ups + t_ups - 2;
  auto const n_mc_techniques = k + 2;
  auto const n_de_techniques = k - 1;
  auto const n_techniques = n_mc_techniques + n_de_techniques;

  auto& p_technique = path_buffer.p_technique;

  for (std::size_t i = 0; i < n_de_techniques; i++) {
    p_technique.emplace_back(p_technique.at(i + 1));
  }

  if (VirtualPerturbation) {
    for (std::size_t i = n_mc_techniques; i < n_techniques; i++) {
      p_technique.at(i) *= camera.ImageSize();
    }
  } else {
    for (std::size_t i = 0; i < n_mc_techniques; i++) {
      p_technique.at(i) /= camera.ImageSize();
    }
  }

  if (VirtualPerturbation) {
    for (std::size_t i = n_mc_techniques; i < n_techniques; i++) {
      p_technique.at(i) /= kernel();
    }
  } else {
    for (std::size_t i = 0; i < n_mc_techniques; i++) {
      p_technique.at(i) *= kernel();
    }
  }

  for (std::size_t i = 0; i < n_de_techniques; i++) {
    auto const p_x_i =
      path_buffer.p_importance.at(i) *
      path_buffer.geometry_factor.at(i);
    if (VirtualPerturbation || s_ups != i + 2) {
      p_technique.at(n_mc_techniques + i) *= p_x_i;
    } else {
      for (std::size_t j = 0; j < n_techniques; j++) {
        if (j == n_mc_techniques + i) {
          continue;
        }
        if (p_x_i == 0) {
          p_technique.at(j) =
            std::numeric_limits<radiant_value_type>::infinity();
        } else {
          p_technique.at(j) /= p_x_i;
        }
      }
    }
  }

  for (std::size_t i = 1; i + 1 + VirtualPerturbation < s_ups; i++) {
    auto const surface = light_path.at(i).object.Surface();
    if (surface != SurfaceType::Diffuse) {
      p_technique.at(i) = 0;
      p_technique.at(i + 1) = 0;
      p_technique.at(n_mc_techniques + i - 1) = 0;
    }
  }

  for (std::size_t i = 1; i + 1 < t_ups; i++) {
    auto const surface = eye_path.at(i).object.Surface();
    if (surface != SurfaceType::Diffuse) {
      p_technique.at(n_mc_techniques - i - 1) = 0;
      p_technique.at(n_mc_techniques - i - 2) = 0;
      p_technique.at(n_techniques - i) = 0;
    }
  }

  return component::PowerHeuristic<radiant_value_type>()(
    p_technique.begin(),
    p_technique.end()
  );
}

}
}
}
