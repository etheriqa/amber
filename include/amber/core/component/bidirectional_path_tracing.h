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
struct SubpathGenerationPolicy
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
  radiant_value_type const
  PDFForward(
    object_type const& object,
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const = 0;

  virtual
  radiant_value_type const
  PDFBackward(
    object_type const& object,
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const = 0;

  virtual
  Radiant const
  BSDFBackward(
    object_type const& object,
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const = 0;
};

template <typename Radiant, typename RealType>
struct LightSubpathGenerationPolicy
: public SubpathGenerationPolicy<Radiant, RealType>
{
  using typename SubpathGenerationPolicy<Radiant, RealType>::object_type;
  using typename SubpathGenerationPolicy<Radiant, RealType>::radiant_value_type;
  using typename SubpathGenerationPolicy<Radiant, RealType>::scatter_type;
  using typename SubpathGenerationPolicy<Radiant, RealType>::unit_vector3_type;

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

  radiant_value_type const
  PDFForward(
    object_type const& object,
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const
  {
    return object.PDFImportance(direction_i, direction_o, normal);
  }

  radiant_value_type const
  PDFBackward(
    object_type const& object,
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const
  {
    return object.PDFLight(direction_i, direction_o, normal);
  }

  Radiant const
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
struct EyeSubpathGenerationPolicy
: public SubpathGenerationPolicy<Radiant, RealType>
{
  using typename SubpathGenerationPolicy<Radiant, RealType>::object_type;
  using typename SubpathGenerationPolicy<Radiant, RealType>::radiant_value_type;
  using typename SubpathGenerationPolicy<Radiant, RealType>::scatter_type;
  using typename SubpathGenerationPolicy<Radiant, RealType>::unit_vector3_type;

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

  radiant_value_type const
  PDFForward(
    object_type const& object,
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const
  {
    return object.PDFLight(direction_i, direction_o, normal);
  }

  radiant_value_type const
  PDFBackward(
    object_type const& object,
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const
  {
    return object.PDFImportance(direction_i, direction_o, normal);
  }

  Radiant const
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
struct SubpathEndpoint
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
struct SubpathEvent : public SubpathEndpoint<Radiant, RealType>
{
  using radiant_value_type = typename Radiant::value_type;

  radiant_value_type geometry_factor;
  radiant_value_type p_backward;
  radiant_value_type p_forward;
};

template <typename Radiant>
struct PathContribution
{
  Radiant measurement;
  boost::optional<std::tuple<std::size_t, std::size_t>> pixel;

  PathContribution() noexcept : measurement(), pixel(boost::none) {}

  PathContribution(Radiant const& measurement) noexcept
  : measurement(measurement), pixel(boost::none) {}

  PathContribution(
    Radiant const& measurement,
    std::tuple<std::size_t, std::size_t> const& pixel
  ) noexcept
  : measurement(measurement), pixel(pixel) {}

  operator bool() const noexcept
  {
    return Max(measurement) > 0;
  }
};

template <typename Radiant, typename RealType>
struct PathBuffer
{
  using camera_type        = Camera<Radiant, RealType>;
  using event_type         = SubpathEvent<Radiant, RealType>;
  using radiant_value_type = typename Radiant::value_type;
  using scene_type         = Scene<Object<Radiant, RealType>>;

  std::vector<radiant_value_type> geometry_factor;
  std::vector<radiant_value_type> p_light;
  std::vector<radiant_value_type> p_importance;
  std::vector<radiant_value_type> p_technique;

  PathBuffer() noexcept
  : geometry_factor()
  , p_light()
  , p_importance()
  , p_technique()
  {}

  void
  Buffer(
    scene_type const& scene,
    camera_type const& camera,
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t const s,
    std::size_t const t
  );

private:
  void
  BufferGeometryFactor(
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t const s,
    std::size_t const t
  );

  void
  BufferLightProbability(
    camera_type const& camera,
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t const s,
    std::size_t const t
  );

  void
  BufferImportanceProbability(
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t const s,
    std::size_t const t
  );

  void
  BufferTechniqueProbability(
    scene_type const& scene,
    camera_type const& camera,
    std::vector<event_type> const& light_path,
    std::vector<event_type> const& eye_path,
    std::size_t const s,
    std::size_t const t
  );
};

template <typename Radiant, typename RealType>
RealType
GeometryFactor(
  SubpathEndpoint<Radiant, RealType> const& x,
  SubpathEndpoint<Radiant, RealType> const& y
) noexcept
{
  return core::GeometryFactor(x.position, y.position, x.normal, y.normal);
}

template <typename Radiant, typename RealType>
std::vector<SubpathEvent<Radiant, RealType>>
GenerateLightSubpath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::size_t const s,
  Sampler& sampler
)
{
  std::vector<SubpathEvent<Radiant, RealType>> events;
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

    SubpathEvent<Radiant, RealType> event;
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

  ExtendSubpath(
    LightSubpathGenerationPolicy<Radiant, RealType>(),
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
std::vector<SubpathEvent<Radiant, RealType>>
GenerateLightSubpath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  Sampler& sampler
)
{
  return GenerateLightSubpath(
    scene,
    camera,
    std::numeric_limits<std::size_t>::max(),
    sampler
  );
}

template <typename Radiant, typename RealType>
std::vector<SubpathEvent<Radiant, RealType>>
GenerateEyeSubpath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::size_t const u,
  std::size_t const v,
  std::size_t const t,
  Sampler& sampler
)
{
  std::vector<SubpathEvent<Radiant, RealType>> events;
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
      camera.GenerateEyeRay(u, v, sampler);

    SubpathEvent<Radiant, RealType> event;
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

  ExtendSubpath(
    EyeSubpathGenerationPolicy<Radiant, RealType>(),
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
std::tuple<
  std::vector<SubpathEvent<Radiant, RealType>>,
  std::size_t,
  std::size_t
>
GenerateEyeSubpath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::size_t const t,
  Sampler& sampler
)
{
  std::vector<SubpathEvent<Radiant, RealType>> events;
  std::size_t u, v;
  if (t == 0) {
    return std::make_tuple(events, u, v);
  }

  events.reserve(8);

  Ray<RealType> ray;
  Radiant weight;
  UnitVector3<RealType> normal;

  {
    Object<Radiant, RealType> object;
    typename Radiant::value_type p_psa;
    std::tie(ray, weight, object, std::ignore, p_psa, normal, u, v) =
      camera.GenerateEyeRay(sampler);

    SubpathEvent<Radiant, RealType> event;
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

  ExtendSubpath(
    EyeSubpathGenerationPolicy<Radiant, RealType>(),
    scene,
    t,
    ray,
    weight,
    normal,
    std::back_inserter(events),
    sampler
  );

  return std::make_tuple(events, u, v);
}


template <typename Radiant, typename RealType>
std::vector<SubpathEvent<Radiant, RealType>>
GenerateEyeSubpath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::size_t const u,
  std::size_t const v,
  Sampler& sampler
)
{
  return GenerateEyeSubpath(
    scene,
    camera,
    u,
    v,
    std::numeric_limits<std::size_t>::max(),
    sampler
  );
}

template <typename Radiant, typename RealType>
std::tuple<
  std::vector<SubpathEvent<Radiant, RealType>>,
  std::size_t,
  std::size_t
>
GenerateEyeSubpath(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  Sampler& sampler
)
{
  return GenerateEyeSubpath(
    scene,
    camera,
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
ExtendSubpath(
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
      std::min<typename Radiant::value_type>(
        kRussianRoulette,
        Max(bsdf / p_backward)
      );
    auto const p_russian_roulette_forward =
      std::min<typename Radiant::value_type>(
        kRussianRoulette,
        Max(scatter.weight)
      );

    auto const geometry_factor = core::GeometryFactor(
      direction_o,
      hit.distance * hit.distance,
      normal,
      hit.normal
    );

    SubpathEvent<Radiant, RealType> event;
    event.object          = object;
    event.position        = hit.position;
    event.normal          = hit.normal;
    event.direction       = direction_o;
    event.weight          = weight;
    event.geometry_factor = geometry_factor;
    event.p_backward      = p_backward * p_russian_roulette_backward;
    event.p_forward       = p_forward * p_russian_roulette_forward;
    output = event;

    if (Uniform<typename Radiant::value_type>(sampler) >=
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

template <typename Radiant, typename RealType>
PathContribution<Radiant>
Connect(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  SubpathEndpoint<Radiant, RealType> const* const light_endpoint,
  SubpathEndpoint<Radiant, RealType> const* const eye_endpoint
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
    return PathContribution<Radiant>(
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

  PathContribution<Radiant> contribution(Radiant(1));

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
void
PathBuffer<Radiant, RealType>::Buffer(
  scene_type const& scene,
  camera_type const& camera,
  std::vector<event_type> const& light_path,
  std::vector<event_type> const& eye_path,
  std::size_t const s,
  std::size_t const t
)
{
  BufferGeometryFactor(light_path, eye_path, s, t);
  BufferLightProbability(camera, light_path, eye_path, s, t);
  BufferImportanceProbability(light_path, eye_path, s, t);
  BufferTechniqueProbability(scene, camera, light_path, eye_path, s, t);
}

template <typename Radiant, typename RealType>
void
PathBuffer<Radiant, RealType>::BufferGeometryFactor(
  std::vector<SubpathEvent<Radiant, RealType>> const& light_path,
  std::vector<SubpathEvent<Radiant, RealType>> const& eye_path,
  std::size_t const s,
  std::size_t const t
)
{
  geometry_factor.clear();

  if (s >= 2) {
    for (std::size_t i = 1; i <= s - 1; i++) {
      geometry_factor.emplace_back(light_path.at(i).geometry_factor);
    }
  }

  if (s >= 1 && t >= 1) {
    geometry_factor.emplace_back(
      GeometryFactor(light_path.at(s - 1), eye_path.at(t - 1))
    );
  }

  if (t >= 2) {
    for (std::size_t i = t - 1; i >= 1; i--) {
      geometry_factor.emplace_back(eye_path.at(i).geometry_factor);
    }
  }
}

template <typename Radiant, typename RealType>
void
PathBuffer<Radiant, RealType>::BufferLightProbability(
  Camera<Radiant, RealType> const& camera,
  std::vector<SubpathEvent<Radiant, RealType>> const& light_path,
  std::vector<SubpathEvent<Radiant, RealType>> const& eye_path,
  std::size_t const s,
  std::size_t const t
)
{
  p_light.clear();

  if (s >= 2) {
    for (std::size_t i = 1; i <= s - 1; i++) {
      p_light.emplace_back(light_path.at(i).p_backward);
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
      std::min<typename Radiant::value_type>(kRussianRoulette, Max(bsdf / pdf));
    p_light.emplace_back(pdf * p_russian_roulette);
  }

  if (t >= 2) {
    for (std::size_t i = t - 2; i <= t - 2; i--) {
      p_light.emplace_back(eye_path.at(i).p_forward);
    }
  } else {
    auto const& z0 = t > 0 ? eye_path.at(0) : light_path.at(s + t - 1);
    auto const& z1 = t > 1 ? eye_path.at(1) : light_path.at(s + t - 2);
    p_light.back() =
      camera.PDFDirection(Normalize(z1.position - z0.position));
  }
}

template <typename Radiant, typename RealType>
void
PathBuffer<Radiant, RealType>::BufferImportanceProbability(
  std::vector<SubpathEvent<Radiant, RealType>> const& light_path,
  std::vector<SubpathEvent<Radiant, RealType>> const& eye_path,
  std::size_t const s,
  std::size_t const t
)
{
  p_importance.clear();

  if (s >= 2) {
    for (std::size_t i = 0; i <= s - 2; i++) {
      p_importance.emplace_back(light_path.at(i).p_forward);
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
      std::min<typename Radiant::value_type>(kRussianRoulette, Max(bsdf / pdf));
    p_importance.emplace_back(pdf * p_russian_roulette);
  }

  if (t >= 2) {
    for (std::size_t i = t - 1; i >= 1; i--) {
      p_importance.emplace_back(eye_path.at(i).p_backward);
    }
  }

  if (s < 2) {
    p_importance.front() = 1 / kPI;
  }
}

template <typename Radiant, typename RealType>
void
PathBuffer<Radiant, RealType>::BufferTechniqueProbability(
  scene_type const& scene,
  camera_type const& camera,
  std::vector<event_type> const& light_path,
  std::vector<event_type> const& eye_path,
  std::size_t const s,
  std::size_t const t
)
{
  auto const n_techniques = s + t + 1;
  p_technique.resize(n_techniques);
  p_technique.at(s) = 1;

  for (std::size_t i = s - 1; i < n_techniques; i--) {
    auto& p = p_technique.at(i);
    p = p_technique.at(i + 1);

    if (i == n_techniques - 2) {
      p *= camera.PDFArea();
    } else {
      p *= p_light.at(i) * geometry_factor.at(i);
    }

    if (i == 0) {
      p /= scene.LightPDFArea(light_path.front().object);
    } else {
      p /= p_importance.at(i - 1) * geometry_factor.at(i - 1);
    }
  }

  for (std::size_t i = s + 1; i < n_techniques; i++) {
    auto& p = p_technique.at(i);
    p = p_technique.at(i - 1);

    if (i == 1) {
      p *= scene.LightPDFArea(eye_path.back().object);
    } else {
      p *= p_importance.at(i - 2) * geometry_factor.at(i - 2);
    }

    if (i == n_techniques - 1) {
      p /= camera.PDFArea();
    } else {
      p /= p_light.at(i - 1) * geometry_factor.at(i - 1);
    }
  }
}

template <typename Radiant, typename RealType, typename MIS>
std::tuple<Radiant, std::vector<PathContribution<Radiant>>>
Combine(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::vector<SubpathEvent<Radiant, RealType>> const& light_path,
  std::vector<SubpathEvent<Radiant, RealType>> const& eye_path,
  PathBuffer<Radiant, RealType>& path_buffer,
  MIS const mis
)
{
  Radiant measurement;
  std::vector<PathContribution<Radiant>> light_image;

  for (std::size_t s = 0; s <= light_path.size(); s++) {
    for (std::size_t t = 0; t <= eye_path.size(); t++) {
      auto contribution = Connect(
        scene,
        camera,
        s == 0 ? nullptr : &light_path.at(s - 1),
        t == 0 ? nullptr : &eye_path.at(t - 1)
      );

      if (!contribution) {
        continue;
      }

      contribution.measurement *= BidirectionalMISWeight(
        scene,
        camera,
        light_path,
        eye_path,
        s,
        t,
        path_buffer,
        mis
      );

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

template <typename Radiant, typename RealType, typename MIS>
typename Radiant::value_type const
BidirectionalMISWeight(
  Scene<Object<Radiant, RealType>> const& scene,
  Camera<Radiant, RealType> const& camera,
  std::vector<SubpathEvent<Radiant, RealType>> const& light_path,
  std::vector<SubpathEvent<Radiant, RealType>> const& eye_path,
  std::size_t const s,
  std::size_t const t,
  PathBuffer<Radiant, RealType>& path_buffer,
  MIS const mis
)
{
  path_buffer.Buffer(scene, camera, light_path, eye_path, s, t);

  auto const n_techniques = s + t + 1;

  for (std::size_t i = 1; i + 1 < s; i++) {
    auto const surface = light_path.at(i).object.Surface();
    if (surface != SurfaceType::Diffuse) {
      path_buffer.p_technique.at(i) = 0;
      path_buffer.p_technique.at(i + 1) = 0;
    }
  }

  for (std::size_t i = 1; i + 1 < t; i++) {
    auto const surface = eye_path.at(i).object.Surface();
    if (surface != SurfaceType::Diffuse) {
      path_buffer.p_technique.at(n_techniques - i - 1) = 0;
      path_buffer.p_technique.at(n_techniques - i - 2) = 0;
    }
  }

  return mis(path_buffer.p_technique.begin(), path_buffer.p_technique.end());
}

}
}
}
