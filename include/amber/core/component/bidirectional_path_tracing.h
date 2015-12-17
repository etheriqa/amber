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
#include <numeric>
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
struct BDPTPathGenerationPolicy
{
  using object_type        = Object<Radiant, RealType>;
  using radiant_value_type = typename Radiant::value_type;
  using scatter_type       = Scatter<Radiant, RealType>;
  using unit_vector3_type  = UnitVector3<RealType>;

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

template <typename Radiant, typename RealType>
struct BDPTLightPathGenerationPolicy
: public BDPTPathGenerationPolicy<Radiant, RealType>
{
  using typename BDPTPathGenerationPolicy<Radiant, RealType>::object_type;
  using typename BDPTPathGenerationPolicy<Radiant, RealType>::radiant_value_type;
  using typename BDPTPathGenerationPolicy<Radiant, RealType>::scatter_type;
  using typename BDPTPathGenerationPolicy<Radiant, RealType>::unit_vector3_type;

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

template <typename Radiant, typename RealType>
struct BDPTEyePathGenerationPolicy
: public BDPTPathGenerationPolicy<Radiant, RealType>
{
  using typename BDPTPathGenerationPolicy<Radiant, RealType>::object_type;
  using typename BDPTPathGenerationPolicy<Radiant, RealType>::radiant_value_type;
  using typename BDPTPathGenerationPolicy<Radiant, RealType>::scatter_type;
  using typename BDPTPathGenerationPolicy<Radiant, RealType>::unit_vector3_type;

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

template <typename Radiant, typename RealType>
struct BDPTEndpoint
{
  using object_type       = Object<Radiant, RealType>;
  using unit_vector3_type = UnitVector3<RealType>;
  using vector3_type      = Vector3<RealType>;

  object_type object;
  vector3_type position;
  unit_vector3_type normal;
  unit_vector3_type direction;
  Radiant weight;
};

template <typename Radiant, typename RealType>
RealType
GeometryFactor(
  BDPTEndpoint<Radiant, RealType> const& x,
  BDPTEndpoint<Radiant, RealType> const& y
) noexcept
{
  return core::GeometryFactor(x.position, y.position, x.normal, y.normal);
}

template <typename Radiant, typename RealType>
struct BDPTEvent : public BDPTEndpoint<Radiant, RealType>
{
  using radiant_value_type = typename Radiant::value_type;

  radiant_value_type geometry_factor;
  radiant_value_type p_backward;
  radiant_value_type p_forward;
};

template <typename Radiant, typename RealType>
std::vector<BDPTEvent<Radiant, RealType>>
GenerateLightPath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::size_t const s,
  Sampler& sampler
)
{
  std::vector<BDPTEvent<Radiant, RealType>> events;
  if (s == 0) {
    return events;
  }

  events.reserve(8);

  Ray<RealType> ray;
  Radiant weight;
  UnitVector3<RealType> normal;

  {
    Object<Radiant, RealType> object;
    typename Radiant::value_type p_psa;
    std::tie(ray, weight, object, std::ignore, p_psa, normal) =
      scene.GenerateLightRay(sampler);

    BDPTEvent<Radiant, RealType> event;
    event.object          = object;
    event.position        = ray.origin;
    event.normal          = normal;
    event.direction       = UnitVector3<RealType>();
    event.weight          = weight;
    event.geometry_factor = std::numeric_limits<RealType>::quiet_NaN();
    event.p_backward      = std::numeric_limits<RealType>::quiet_NaN();
    event.p_forward       = p_psa;
    events.push_back(event);
  }

  ExtendPath(
    BDPTLightPathGenerationPolicy<Radiant, RealType>(),
    scene,
    s,
    ray,
    weight,
    normal,
    std::back_inserter(events),
    sampler
  );

  return events;
}

template <typename Radiant, typename RealType>
std::vector<BDPTEvent<Radiant, RealType>>
GenerateLightPath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  Sampler& sampler
)
{
  return GenerateLightPath(
    scene,
    camera,
    std::numeric_limits<std::size_t>::max(),
    sampler
  );
}

template <typename Radiant, typename RealType>
std::vector<BDPTEvent<Radiant, RealType>>
GenerateEyePath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::size_t const x,
  std::size_t const y,
  std::size_t const t,
  Sampler& sampler
)
{
  std::vector<BDPTEvent<Radiant, RealType>> events;
  if (t == 0) {
    return events;
  }

  events.reserve(8);

  Ray<RealType> ray;
  Radiant weight;
  UnitVector3<RealType> normal;

  {
    Object<Radiant, RealType> object;
    typename Radiant::value_type p_psa;
    std::tie(ray, weight, object, std::ignore, p_psa, normal) =
      camera.GenerateEyeRay(x, y, sampler);

    BDPTEvent<Radiant, RealType> event;
    event.object          = object;
    event.position        = ray.origin;
    event.normal          = normal;
    event.direction       = UnitVector3<RealType>();
    event.weight          = weight;
    event.geometry_factor = std::numeric_limits<RealType>::quiet_NaN();
    event.p_backward      = std::numeric_limits<RealType>::quiet_NaN();
    event.p_forward       = p_psa;
    events.push_back(event);
  }

  ExtendPath(
    BDPTEyePathGenerationPolicy<Radiant, RealType>(),
    scene,
    t,
    ray,
    weight,
    normal,
    std::back_inserter(events),
    sampler
  );

  return events;
}

template <typename Radiant, typename RealType>
std::vector<BDPTEvent<Radiant, RealType>>
GenerateEyePath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::size_t const x,
  std::size_t const y,
  Sampler& sampler
)
{
  return GenerateEyePath(
    scene,
    camera,
    x,
    y,
    std::numeric_limits<std::size_t>::max(),
    sampler
  );
}

template <
  typename Policy,
  typename Radiant,
  typename RealType,
  typename OutputIterator
>
void
ExtendPath(
  Policy const policy,
  Scene<Object<Radiant, RealType>> const& scene,
  std::size_t const n_vertices,
  Ray<RealType>& ray,
  Radiant& weight,
  UnitVector3<RealType>& normal,
  OutputIterator output,
  Sampler& sampler
)
{
  for (std::size_t i = 1; i < n_vertices; i++) {
    Hit<RealType> hit;
    Object<Radiant, RealType> object;
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
      std::min<decltype(Max(bsdf / p_backward))>(1, Max(bsdf / p_backward));
    auto const p_russian_roulette_forward =
      std::min<decltype(Max(scatter.weight))>(1, Max(scatter.weight));

    auto const geometry_factor = core::GeometryFactor(
      direction_o,
      hit.distance * hit.distance,
      normal,
      hit.normal
    );

    BDPTEvent<Radiant, RealType> event;
    event.object          = object;
    event.position        = hit.position;
    event.normal          = hit.normal;
    event.direction       = direction_o;
    event.weight          = weight;
    event.geometry_factor = geometry_factor;
    event.p_backward      = p_backward * p_russian_roulette_backward;
    event.p_forward       = p_forward * p_russian_roulette_forward;
    output = event;

    if (Uniform<decltype(p_russian_roulette_forward)>(sampler) >=
        p_russian_roulette_forward
    ) {
      break;
    }

    ray.origin = hit.position;
    ray.direction = scatter.direction;
    normal = hit.normal;
    weight *= scatter.weight / p_russian_roulette_forward;
  }
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
BDPTContribution<Radiant>
Connect(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  BDPTEndpoint<Radiant, RealType> const* const light_endpoint,
  BDPTEndpoint<Radiant, RealType> const* const eye_endpoint
)
{
  if (light_endpoint == nullptr && eye_endpoint == nullptr) {
    return Radiant();
  }

  if (light_endpoint == nullptr) {
    // s = 0; the eye subpath intersects a light source
    if (eye_endpoint->object.Surface() != SurfaceType::Light) {
      return Radiant();
    }
    return
      eye_endpoint->object.Radiance(
        eye_endpoint->direction,
        eye_endpoint->normal
      ) *
      eye_endpoint->weight;
  }

  if (eye_endpoint == nullptr) {
    // t = 0; the light subpath intersects the aperture
    if (light_endpoint->object.Surface() != SurfaceType::Eye) {
      return Radiant();
    }
    auto const response =
      camera.Response(light_endpoint->direction, light_endpoint->position);
    if (!response) {
      return Radiant();
    }
    return BDPTContribution<Radiant>(
      std::get<2>(*response) * light_endpoint->weight,
      std::make_tuple(std::get<0>(*response), std::get<1>(*response))
    );
  }

  {
    auto const surface = light_endpoint->object.Surface();
    if (surface == SurfaceType::Specular || surface == SurfaceType::Eye) {
      return Radiant();
    }
  }

  {
    auto const surface = eye_endpoint->object.Surface();
    if (surface == SurfaceType::Specular ||
        (SquaredLength(eye_endpoint->direction) > 0 &&
         surface == SurfaceType::Eye)) {
      return Radiant();
    }
  }

  BDPTContribution<Radiant> contribution(Radiant(1));

  Ray<RealType> const ray(
    light_endpoint->position,
    eye_endpoint->position - light_endpoint->position
  );

  if (SquaredLength(light_endpoint->direction) == 0) {
    // s = 1
    if (Dot(ray.direction, light_endpoint->normal) <= 0) {
      return Radiant();
    }
    contribution.measurement *=
      light_endpoint->object.Radiance(ray.direction, light_endpoint->normal) /
      scene.LightPDFArea(light_endpoint->object);
  } else {
    // s > 1
    contribution.measurement *=
      light_endpoint->weight *
      light_endpoint->object.AdjointBSDF(
        ray.direction,
        light_endpoint->direction,
        light_endpoint->normal
      );
  }

  if (SquaredLength(eye_endpoint->direction) == 0) {
    // t = 1
    auto const response =
      camera.Response(-ray.direction, eye_endpoint->position);
    if (!response) {
      return Radiant();
    }
    contribution.measurement *= std::get<2>(*response) / camera.PDFArea();
    contribution.pixel =
      std::make_tuple(std::get<0>(*response), std::get<1>(*response));
  } else {
    // t > 1
    contribution.measurement *=
      eye_endpoint->weight *
      eye_endpoint->object.BSDF(
        -ray.direction,
        eye_endpoint->direction,
        eye_endpoint->normal
      );
  }

  if (!scene.TestVisibility(ray, eye_endpoint->object)) {
    return Radiant();
  }

  contribution.measurement *= GeometryFactor(*light_endpoint, *eye_endpoint);

  return contribution;
}

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

  template <typename MIS>
  std::tuple<Radiant, std::vector<contribution_type>>
  Combine(
    scene_type const& scene,
    camera_type const& camera,
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    MIS const mis
  ) const
  {
    Radiant measurement;
    std::vector<contribution_type> light_image;

    for (std::size_t s = 0; s <= light_path.size(); s++) {
      for (std::size_t t = 0; t <= eye_path.size(); t++) {
        auto contribution = component::Connect(
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
          contribution.measurement /= camera.ImageSize();
          light_image.emplace_back(contribution);
        } else {
          measurement += contribution.measurement;
        }
      }
    }

    return std::make_tuple(measurement, light_image);
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
    std::lock_guard<std::mutex> lock(buffer_mtx_);

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

private:
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
