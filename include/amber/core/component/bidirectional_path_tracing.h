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
#include <tuple>
#include <vector>

#include <boost/optional.hpp>

#include "core/camera.h"
#include "core/constant.h"
#include "core/geometry.h"
#include "core/scene.h"

namespace amber {
namespace core {
namespace component {

template <typename Radiant, typename RealType>
struct BDPTEvent
{
  using object_type        = Object<Radiant, RealType>;
  using radiant_value_type = typename Radiant::value_type;
  using unit_vector3_type  = UnitVector3<RealType>;
  using vector3_type       = Vector3<RealType>;

  object_type object;
  vector3_type position;
  unit_vector3_type normal;
  unit_vector3_type direction;
  Radiant weight;
  radiant_value_type geometry_factor;
  radiant_value_type p_backward;
  radiant_value_type p_forward;
};

template <typename Radiant, typename RealType>
RealType
GeometryFactor(
  BDPTEvent<Radiant, RealType> const& x,
  BDPTEvent<Radiant, RealType> const& y
) noexcept
{
  return core::GeometryFactor(x.position, y.position, x.normal, y.normal);
}

template <typename Radiant>
struct BDPTContribution
{
  Radiant measurement;
  boost::optional<std::tuple<std::size_t, std::size_t>> pixel;

  BDPTContribution() noexcept : measurement(), pixel(boost::none) {}

  BDPTContribution(Radiant const& measurement) noexcept
  : measurement(measurement), pixel(boost::none) {}

  BDPTContribution(
    Radiant const& measurement,
    std::tuple<std::size_t, std::size_t> const& pixel
  ) noexcept
  : measurement(measurement), pixel(pixel) {}
};

template <typename Radiant, typename RealType>
class BidirectionalPathTracing
{
public:
  using event_type        = BDPTEvent<Radiant, RealType>;
  using contribution_type = BDPTContribution<Radiant>;

  using camera_type = Camera<Radiant, RealType>;
  using scene_type  = Scene<Object<Radiant, RealType>>;

private:
  using hit_type           = Hit<RealType>;
  using object_type        = Object<Radiant, RealType>;
  using radiant_value_type = typename Radiant::value_type;
  using ray_type           = Ray<RealType>;
  using scatter_type       = Scatter<Radiant, RealType>;
  using unit_vector3_type  = UnitVector3<RealType>;

private:
  struct PathGenerationPolicy
  {
    virtual
    scatter_type
    Scatter(
      object_type const& object,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal,
      Sampler& sampler
    ) const = 0;

    virtual
    radiant_value_type
    PDFForward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const = 0;

    virtual
    radiant_value_type
    PDFBackward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const = 0;

    virtual
    Radiant
    BSDFBackward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const = 0;
  };

  struct LightPathGenerationPolicy : public PathGenerationPolicy
  {
    scatter_type
    Scatter(
      object_type const& object,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal,
      Sampler& sampler
    ) const
    {
      return object.SampleImportance(direction_o, normal, sampler);
    }

    radiant_value_type
    PDFForward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const
    {
      return object.PDFImportance(direction_i, direction_o, normal);
    }

    radiant_value_type
    PDFBackward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const
    {
      return object.PDFLight(direction_i, direction_o, normal);
    }

    Radiant
    BSDFBackward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const
    {
      return object.BSDF(direction_i, direction_o, normal);
    }
  };

  struct EyePathGenerationPolicy : public PathGenerationPolicy
  {
    scatter_type
    Scatter(
      object_type const& object,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal,
      Sampler& sampler
    ) const
    {
      return object.SampleLight(direction_o, normal, sampler);
    }

    radiant_value_type
    PDFForward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const
    {
      return object.PDFLight(direction_i, direction_o, normal);
    }

    radiant_value_type
    PDFBackward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const
    {
      return object.PDFImportance(direction_i, direction_o, normal);
    }

    Radiant
    BSDFBackward(
      object_type const& object,
      unit_vector3_type const& direction_i,
      unit_vector3_type const& direction_o,
      unit_vector3_type const& normal
    ) const
    {
      return object.AdjointBSDF(direction_i, direction_o, normal);
    }
  };

  std::mutex                      mutable buffer_mtx_;
  std::vector<radiant_value_type> mutable buffer_geometry_factor_;
  std::vector<radiant_value_type> mutable buffer_p_light_;
  std::vector<radiant_value_type> mutable buffer_p_importance_;
  std::vector<radiant_value_type> mutable buffer_probability_;

public:
  BidirectionalPathTracing() noexcept
  : buffer_mtx_(),
    buffer_geometry_factor_(),
    buffer_p_light_(),
    buffer_p_importance_(),
    buffer_probability_()
  {}

  std::vector<event_type>
  GenerateLightPath(
    scene_type const& scene,
    camera_type const& camera,
    Sampler& sampler
  ) const
  {
    std::vector<event_type> events;
    ray_type ray;
    Radiant weight;
    unit_vector3_type normal;

    {
      object_type object;
      radiant_value_type p_psa;
      std::tie(ray, weight, object, std::ignore, p_psa, normal) =
        scene.GenerateLightRay(sampler);

      event_type event;
      event.object          = object;
      event.position        = ray.origin;
      event.normal          = normal;
      event.direction       = unit_vector3_type();
      event.weight          = weight;
      event.geometry_factor = std::numeric_limits<RealType>::quiet_NaN();
      event.p_backward      = std::numeric_limits<RealType>::quiet_NaN();
      event.p_forward       = p_psa;
      events.push_back(event);
    }

    ExtendPath(
      LightPathGenerationPolicy(),
      scene,
      ray,
      weight,
      normal,
      std::back_inserter(events),
      sampler
    );

    return events;
  }

  std::vector<event_type>
  GenerateEyePath(
    scene_type const& scene,
    camera_type const& camera,
    std::size_t x,
    std::size_t y,
    Sampler& sampler
  ) const
  {
    std::vector<event_type> events;
    ray_type ray;
    Radiant weight;
    unit_vector3_type normal;

    {
      object_type object;
      radiant_value_type p_psa;
      std::tie(ray, weight, object, std::ignore, p_psa, normal) =
        camera.GenerateEyeRay(x, y, sampler);

      event_type event;
      event.object          = object;
      event.position        = ray.origin;
      event.normal          = normal;
      event.direction       = unit_vector3_type();
      event.weight          = weight;
      event.geometry_factor = std::numeric_limits<RealType>::quiet_NaN();
      event.p_backward      = std::numeric_limits<RealType>::quiet_NaN();
      event.p_forward       = p_psa;
      events.push_back(event);
    }

    ExtendPath(
      EyePathGenerationPolicy(),
      scene,
      ray,
      weight,
      normal,
      std::back_inserter(events),
      sampler
    );

    return events;
  }

  template <typename MIS>
  std::tuple<Radiant, std::vector<contribution_type>>
  Connect(
    scene_type const& scene,
    camera_type const& camera,
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    MIS mis
  ) const
  {
    std::lock_guard<std::mutex> lock(buffer_mtx_);

    Radiant measurement;
    std::vector<contribution_type> light_image;

    for (std::size_t s = 0; s <= light_path.size(); s++) {
      for (std::size_t t = 0; t <= eye_path.size(); t++) {
        auto contribution = UnweightedContribution(
          scene,
          camera,
          s == 0 ? nullptr : &light_path.at(s - 1),
          t == 0 ? nullptr : &eye_path.at(t - 1)
        );

        if (Max(contribution.measurement) == 0) {
          continue;
        }

        contribution.measurement *=
          MISWeight(scene, camera, light_path, eye_path, s, t, mis);

        if (contribution.pixel) {
          contribution.measurement /= camera.imageSize();
          light_image.emplace_back(contribution);
        } else {
          measurement += contribution.measurement;
        }
      }
    }

    return std::make_tuple(measurement, light_image);
  }

private:
  template <typename OutputIterator>
  void
  ExtendPath(
    PathGenerationPolicy const& policy,
    scene_type const& scene,
    ray_type& ray,
    Radiant& weight,
    unit_vector3_type& normal,
    OutputIterator output,
    Sampler& sampler
  ) const
  {
    for (;;) {
      hit_type hit;
      object_type object;
      std::tie(hit, object) = scene.Cast(ray);
      if (!hit) {
        break;
      }

      auto const scatter =
        policy.Scatter(object, -ray.direction, hit.normal, sampler);
      auto const direction_o = -ray.direction;
      auto const& direction_i = scatter.direction;

      auto const p_forward =
        policy.PDFForward(object, direction_i, direction_o, hit.normal);
      auto const p_backward =
        policy.PDFBackward(object, direction_o, direction_i, hit.normal);
      auto const bsdf =
        policy.BSDFBackward(object, direction_o, direction_i, hit.normal);
      auto const p_russian_roulette_backward =
        std::min<radiant_value_type>(1, Max(bsdf / p_backward));
      auto const p_russian_roulette_forward =
        std::min<radiant_value_type>(1, Max(scatter.weight));

      auto const geometry_factor = core::GeometryFactor(
        direction_o,
        hit.distance * hit.distance,
        normal,
        hit.normal
      );

      event_type event;
      event.object          = object;
      event.position        = hit.position;
      event.normal          = hit.normal;
      event.direction       = direction_o;
      event.weight          = weight;
      event.geometry_factor = geometry_factor;
      event.p_backward      = p_backward * p_russian_roulette_backward;
      event.p_forward       = p_forward * p_russian_roulette_forward;
      output = event;

      if (Uniform<radiant_value_type>(sampler) >= p_russian_roulette_forward) {
        break;
      }

      ray = ray_type(hit.position, scatter.direction);
      normal = hit.normal;
      weight *= scatter.weight / p_russian_roulette_forward;
    }
  }

  contribution_type
  UnweightedContribution(
    scene_type const& scene,
    camera_type const& camera,
    event_type const* const light_end,
    event_type const* const eye_end
  ) const
  {
    if (light_end == nullptr && eye_end == nullptr) {
      return contribution_type();
    }

    if (light_end == nullptr) {
      // s = 0; the eye subpath intersects a light source
      if (eye_end->object.Surface() != SurfaceType::Light) {
        return contribution_type();
      }
      return
        eye_end->object.Radiance(eye_end->direction, eye_end->normal) *
        eye_end->weight;
    }

    if (eye_end == nullptr) {
      // t = 0; the light subpath intersects the aperture
      if (light_end->object.Surface() != SurfaceType::Eye) {
        return contribution_type();
      }
      auto const response =
        camera.Response(light_end->direction, light_end->position);
      if (!response) {
        return contribution_type();
      }
      return contribution_type(
        std::get<2>(*response) * light_end->weight,
        std::make_tuple(std::get<0>(*response), std::get<1>(*response))
      );
    }

    {
      auto const surface = light_end->object.Surface();
      if (surface == SurfaceType::Specular || surface == SurfaceType::Eye) {
        return contribution_type();
      }
    }

    {
      auto const surface = eye_end->object.Surface();
      if (surface == SurfaceType::Specular ||
          (eye_end->geometry_factor > 0 && surface == SurfaceType::Eye)) {
        return contribution_type();
      }
    }

    contribution_type contribution(Radiant(1));

    ray_type const ray(
      light_end->position,
      eye_end->position - light_end->position
    );

    if (std::isnan(light_end->geometry_factor)) {
      // s = 1
      if (Dot(ray.direction, light_end->normal) <= 0) {
        return contribution_type();
      }
      contribution.measurement *=
        light_end->object.Radiance(ray.direction, light_end->normal) /
        scene.LightPDFArea(light_end->object);
    } else {
      // s > 1
      contribution.measurement *=
        light_end->weight *
        light_end->object.AdjointBSDF(
          ray.direction,
          light_end->direction,
          light_end->normal
        );
    }

    if (std::isnan(eye_end->geometry_factor)) {
      // t = 1
      auto const response = camera.Response(-ray.direction, eye_end->position);
      if (!response) {
        return contribution_type();
      }
      contribution.measurement *= std::get<2>(*response) / camera.PDFArea();
      contribution.pixel =
        std::make_tuple(std::get<0>(*response), std::get<1>(*response));
    } else {
      // t > 1
      contribution.measurement *=
        eye_end->weight *
        eye_end->object.BSDF(
          -ray.direction,
          eye_end->direction,
          eye_end->normal
        );
    }

    if (!scene.TestVisibility(ray, eye_end->object)) {
      return contribution_type();
    }

    contribution.measurement *= GeometryFactor(*light_end, *eye_end);

    return contribution;
  }

  template <typename MIS>
  radiant_value_type
  MISWeight(
    scene_type const& scene,
    camera_type const& camera,
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t s,
    std::size_t t,
    MIS mis
  ) const
  {
    auto const n_techniques = s + t + 1;

    UpdateGeometryFactorBuffer(light_path, eye_path, s, t);
    UpdateLightProbabilityBuffer(camera, light_path, eye_path, s, t);
    UpdateImportanceProbabilityBuffer(light_path, eye_path, s, t);

    buffer_probability_.resize(n_techniques);
    buffer_probability_.at(s) = 1;

    for (std::size_t i = s - 1; i < n_techniques; i--) {
      auto& p = buffer_probability_.at(i);
      p = buffer_probability_.at(i + 1);

      if (i == n_techniques - 2) {
        p *= camera.PDFArea();
      } else {
        p *= buffer_p_light_.at(i) * buffer_geometry_factor_.at(i);
      }

      if (i == 0) {
        p /= scene.LightPDFArea(light_path.front().object);
      } else {
        p /= buffer_p_importance_.at(i - 1) * buffer_geometry_factor_.at(i - 1);
      }
    }

    for (std::size_t i = s + 1; i < n_techniques; i++) {
      auto& p = buffer_probability_.at(i);
      p = buffer_probability_.at(i - 1);

      if (i == 1) {
        p *= scene.LightPDFArea(eye_path.back().object);
      } else {
        p *= buffer_p_importance_.at(i - 2) * buffer_geometry_factor_.at(i - 2);
      }

      if (i == n_techniques - 1) {
        p /= camera.PDFArea();
      } else {
        p /= buffer_p_light_.at(i - 1) * buffer_geometry_factor_.at(i - 1);
      }
    }

    for (std::size_t i = 1; i + 1 < s; i++) {
      auto const surface = light_path.at(i).object.Surface();
      if (surface == SurfaceType::Specular || surface == SurfaceType::Eye) {
        buffer_probability_.at(i) = 0;
        buffer_probability_.at(i + 1) = 0;
      }
    }
    for (std::size_t i = 1; i + 1 < t; i++) {
      auto const surface = eye_path.at(i).object.Surface();
      if (surface == SurfaceType::Specular || surface == SurfaceType::Eye) {
        buffer_probability_.at(n_techniques - i - 1) = 0;
        buffer_probability_.at(n_techniques - i - 2) = 0;
      }
    }

    return mis(buffer_probability_.begin(), buffer_probability_.end());
  }

  void
  UpdateGeometryFactorBuffer(
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t s,
    std::size_t t
  ) const
  {
    buffer_geometry_factor_.clear();

    if (s >= 2) {
      for (std::size_t i = 1; i <= s - 1; i++) {
        buffer_geometry_factor_.push_back(light_path.at(i).geometry_factor);
      }
    }

    if (s >= 1 && t >= 1) {
      buffer_geometry_factor_.push_back(
        GeometryFactor(light_path.at(s - 1), eye_path.at(t - 1))
      );
    }

    if (t >= 2) {
      for (std::size_t i = t - 1; i >= 1; i--) {
        buffer_geometry_factor_.push_back(eye_path.at(i).geometry_factor);
      }
    }
  }

  void
  UpdateLightProbabilityBuffer(
    camera_type const& camera,
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t s,
    std::size_t t
  ) const
  {
    buffer_p_light_.clear();

    if (s >= 2) {
      for (std::size_t i = 1; i <= s - 1; i++) {
        buffer_p_light_.push_back(light_path.at(i).p_backward);
      }
    }

    if (s >= 1 && t >= 1) {
      auto const& light = light_path.at(s - 1);
      auto const& eye = eye_path.at(t - 1);
      auto const incident = Normalize(light.position - eye.position);
      auto const& exitant = eye.direction;
      auto const& normal = eye.normal;
      auto const bsdf = eye.object.BSDF(incident, exitant, normal);
      auto const pdf = eye.object.PDFLight(incident, exitant, normal);
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, Max(bsdf / pdf));
      buffer_p_light_.push_back(pdf * p_russian_roulette);
    }

    if (t >= 2) {
      for (std::size_t i = t - 2; i <= t - 2; i--) {
        buffer_p_light_.push_back(eye_path.at(i).p_forward);
      }
    } else {
      auto const& z0 = t > 0 ? eye_path.at(0) : light_path.at(s + t - 1);
      auto const& z1 = t > 1 ? eye_path.at(1) : light_path.at(s + t - 2);
      buffer_p_light_.back() =
        camera.PDFDirection(Normalize(z1.position - z0.position));
    }
  }

  void
  UpdateImportanceProbabilityBuffer(
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t s,
    std::size_t t
  ) const
  {
    buffer_p_importance_.clear();

    if (s >= 2) {
      for (std::size_t i = 0; i <= s - 2; i++) {
        buffer_p_importance_.push_back(light_path.at(i).p_forward);
      }
    }

    if (s >= 1 && t >= 1) {
      auto const& light = light_path.at(s - 1);
      auto const& eye = eye_path.at(t - 1);
      auto const incident = Normalize(eye.position - light.position);
      auto const& exitant = light.direction;
      auto const& normal = light.normal;
      auto const bsdf = light.object.AdjointBSDF(incident, exitant, normal);
      auto const pdf = light.object.PDFImportance(incident, exitant, normal);
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, Max(bsdf / pdf));
      buffer_p_importance_.push_back(pdf * p_russian_roulette);
    }

    if (t >= 2) {
      for (std::size_t i = t - 1; i >= 1; i--) {
        buffer_p_importance_.push_back(eye_path.at(i).p_backward);
      }
    }

    if (s < 2) {
      // XXX
      buffer_p_importance_.front() = 1 / kPI;
    }
  }
};

}
}
}
