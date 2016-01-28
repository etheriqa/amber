// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include <cmath>
#include <algorithm>
#include <iterator>
#include <limits>
#include <vector>

#include "amber/prelude/geometry.h"
#include "amber/prelude/ray.h"
#include "amber/prelude/sampling.h"
#include "amber/prelude/vector3.h"
#include "amber/rendering/forward.h"
#include "amber/rendering/leading.h"
#include "amber/rendering/object.h"
#include "amber/rendering/scatter.h"
#include "amber/rendering/scene.h"
#include "amber/rendering/sensor.h"
#include "amber/rendering/surface.h"

namespace amber {
namespace rendering {

/** Each event (vertex) in subpath.
 */
template <typename Radiant>
class SubpathEvent
{
public:
  /** Constructors.
   */
  SubpathEvent(
    const ObjectPointer& object,
    const Leading<Radiant>& leading
  ) noexcept;
  SubpathEvent(
    const ObjectPointer& object,
    const Vector3& position,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const Radiant& weight,
    real_type geometry_factor,
    real_type p_backward,
    real_type p_forward
  ) noexcept;

  /** Queries.
   */
  bool IsLeading() const noexcept;
  ObjectPointer Object() const noexcept;
  const Vector3 Position() const noexcept;
  const UnitVector3 Normal() const noexcept;
  const UnitVector3 DirectionOut() const noexcept;
  const Radiant Weight() const noexcept;
  const real_type GeometryFactor() const noexcept;
  const real_type BackwardPDF() const noexcept;
  const real_type ForwardPDF() const noexcept;

private:
  ObjectPointer object_;
  Vector3 position_;
  UnitVector3 normal_;
  UnitVector3 direction_out_;
  Radiant weight_;
  real_type geometry_factor_;
  real_type p_backward_;
  real_type p_forward_;
};

/** Subpath.
 */
template <typename Radiant>
class Subpath
{
public:
  using Container          = std::vector<SubpathEvent<Radiant>>;
  using ConstIterator      = typename Container::const_iterator;
  using BackInsertIterator = std::back_insert_iterator<Container>;

  /** Constructor.
   */
  Subpath() noexcept;

  /** Iterators.
   */
  ConstIterator begin() const noexcept;
  ConstIterator end() const noexcept;
  operator BackInsertIterator();

  /** Queries.
   */
  const path_size_type Size() const noexcept;

private:
  Container events_;
};

/** Buffer object for the purpose of efficient calculation of MIS weight.
 */
template <typename Radiant>
class BidirectionalPathSamplingBuffer
{
public:
  using Container            = std::vector<real_type>;
  using SubpathConstIterator = typename Subpath<Radiant>::ConstIterator;

  /** Constructor.
   */
  BidirectionalPathSamplingBuffer() noexcept;

  /** Buffers full path information.
   */
  void
  Buffer(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    SubpathConstIterator light_first,
    SubpathConstIterator light_last,
    SubpathConstIterator eye_first,
    SubpathConstIterator eye_last
  );

  /** Calculates MIS weight for the current buffer.
   */
  const real_type MISWeight(const MIS& mis) const noexcept;

protected:
  std::vector<real_type> geometry_factor_;
  std::vector<real_type> p_light_;
  std::vector<real_type> p_importance_;
  std::vector<real_type> p_technique_;

  void
  BufferGeometryFactor(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    SubpathConstIterator light_first,
    SubpathConstIterator light_last,
    SubpathConstIterator eye_first,
    SubpathConstIterator eye_last
  );
  void
  BufferLightProbability(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    SubpathConstIterator light_first,
    SubpathConstIterator light_last,
    SubpathConstIterator eye_first,
    SubpathConstIterator eye_last
  );
  void BufferImportanceProbability(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    SubpathConstIterator light_first,
    SubpathConstIterator light_last,
    SubpathConstIterator eye_first,
    SubpathConstIterator eye_last
  );
  void
  BufferTechniqueProbability(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    SubpathConstIterator light_first,
    SubpathConstIterator light_last,
    SubpathConstIterator eye_first,
    SubpathConstIterator eye_last
  );
  void
  DisableSpecularConnection(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    SubpathConstIterator light_first,
    SubpathConstIterator light_last,
    SubpathConstIterator eye_first,
    SubpathConstIterator eye_last
  );
};

/** Calculates the geometry factor between two vertices.
 */
template <typename Radiant>
const real_type
GeometryFactor(
  const SubpathEvent<Radiant>& x,
  const SubpathEvent<Radiant>& y
) noexcept;

/** Generates a light subpath.
 */
template <typename Radiant>
void
GenerateLightSubpath(
  const Scene<Radiant>& scene,
  Sampler& sampler,
  typename Subpath<Radiant>::BackInsertIterator inserter
);

/** Generates a light subpath that has a specified number of events.
 */
template <typename Radiant>
void
GenerateLightSubpath(
  const Scene<Radiant>& scene,
  Sampler& sampler,
  typename Subpath<Radiant>::BackInsertIterator inserter,
  path_size_type s
);

/** Generates an eye subpath.
 */
template <typename Radiant>
Pixel
GenerateEyeSubpath(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  Sampler& sampler,
  typename Subpath<Radiant>::BackInsertIterator inserter
);

/** Generates an eye subpath that has a specified number of events.
 */
template <typename Radiant>
Pixel
GenerateEyeSubpath(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  Sampler& sampler,
  typename Subpath<Radiant>::BackInsertIterator inserter,
  path_size_type t
);

/** Connects a light and an eye endpoint into a full path.
 */
template <typename Radiant>
PixelValue<Radiant>
Connect(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  const SubpathEvent<Radiant>* light_endpoint,
  const SubpathEvent<Radiant>* eye_endpoint
);

/** Combines a light and an eye subpath with multiple importance sampling (MIS).
 */
template <typename Radiant>
const Radiant
Combine(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  const Subpath<Radiant>& light_path,
  const Subpath<Radiant>& eye_path,
  const MIS& mis,
  BidirectionalPathSamplingBuffer<Radiant>& buffer,
  SparseImage<Radiant>& light_image
);





template <typename Radiant>
SubpathEvent<Radiant>::SubpathEvent(
  const ObjectPointer& object,
  const Leading<Radiant>& leading
) noexcept
: object_(object)
, position_(leading.Position())
, normal_(leading.Normal())
, direction_out_(leading.Normal())
, weight_(leading.Weight())
, geometry_factor_(std::numeric_limits<real_type>::quiet_NaN())
, p_backward_(std::numeric_limits<real_type>::quiet_NaN())
, p_forward_(std::numeric_limits<real_type>::quiet_NaN())
{}

template <typename Radiant>
SubpathEvent<Radiant>::SubpathEvent(
  const ObjectPointer& object,
  const Vector3& position,
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const Radiant& weight,
  real_type geometry_factor,
  real_type p_backward,
  real_type p_forward
) noexcept
: object_(object)
, position_(position)
, normal_(normal)
, direction_out_(direction_out)
, weight_(weight)
, geometry_factor_(geometry_factor)
, p_backward_(p_backward)
, p_forward_(p_forward)
{}

template <typename Radiant>
bool
SubpathEvent<Radiant>::IsLeading() const noexcept
{
  return std::isnan(geometry_factor_);
}

template <typename Radiant>
ObjectPointer
SubpathEvent<Radiant>::Object() const noexcept
{
  return object_;
}

template <typename Radiant>
const Vector3
SubpathEvent<Radiant>::Position() const noexcept
{
  return position_;
}

template <typename Radiant>
const UnitVector3
SubpathEvent<Radiant>::Normal() const noexcept
{
  return normal_;
}

template <typename Radiant>
const UnitVector3
SubpathEvent<Radiant>::DirectionOut() const noexcept
{
  return direction_out_;
}

template <typename Radiant>
const Radiant
SubpathEvent<Radiant>::Weight() const noexcept
{
  return weight_;
}

template <typename Radiant>
const real_type
SubpathEvent<Radiant>::GeometryFactor() const noexcept
{
  return geometry_factor_;
}

template <typename Radiant>
const real_type
SubpathEvent<Radiant>::BackwardPDF() const noexcept
{
  return p_backward_;
}

template <typename Radiant>
const real_type
SubpathEvent<Radiant>::ForwardPDF() const noexcept
{
  return p_forward_;
}

template <typename Radiant>
Subpath<Radiant>::Subpath() noexcept
: events_()
{}

template <typename Radiant>
auto
Subpath<Radiant>::begin() const noexcept
-> ConstIterator
{
  return events_.begin();
}

template <typename Radiant>
auto
Subpath<Radiant>::end() const noexcept
-> ConstIterator
{
  return events_.end();
}

template <typename Radiant>
Subpath<Radiant>::operator BackInsertIterator()
{
  events_.clear();
  return std::back_inserter(events_);
}

template <typename Radiant>
const path_size_type
Subpath<Radiant>::Size() const noexcept
{
  return events_.size();
}

template <typename Radiant>
BidirectionalPathSamplingBuffer<Radiant>
::BidirectionalPathSamplingBuffer() noexcept
: geometry_factor_()
, p_light_()
, p_importance_()
, p_technique_()
{}

template <typename Radiant>
void
BidirectionalPathSamplingBuffer<Radiant>
::Buffer(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  SubpathConstIterator light_first,
  SubpathConstIterator light_last,
  SubpathConstIterator eye_first,
  SubpathConstIterator eye_last
)
{
  BufferGeometryFactor(
    scene, sensor, light_first, light_last, eye_first, eye_last);
  BufferLightProbability(
    scene, sensor, light_first, light_last, eye_first, eye_last);
  BufferImportanceProbability(
    scene, sensor, light_first, light_last, eye_first, eye_last);
  BufferTechniqueProbability(
    scene, sensor, light_first, light_last, eye_first, eye_last);
  DisableSpecularConnection(
    scene, sensor, light_first, light_last, eye_first, eye_last);
}

template <typename Radiant>
const real_type
BidirectionalPathSamplingBuffer<Radiant>
::MISWeight(const MIS& mis) const noexcept
{
  return mis(p_technique_);
}

template <typename Radiant>
void
BidirectionalPathSamplingBuffer<Radiant>
::BufferGeometryFactor(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  SubpathConstIterator light_first,
  SubpathConstIterator light_last,
  SubpathConstIterator eye_first,
  SubpathConstIterator eye_last
)
{
  geometry_factor_.clear();

  if (std::distance(light_first, light_last) >= 2) {
    std::transform(
      light_first + 1,
      light_last,
      std::back_inserter(geometry_factor_),
      [](const auto& event){ return event.GeometryFactor(); }
    );
  }

  if (light_first != light_last && eye_first != eye_last) {
    geometry_factor_.emplace_back(
      GeometryFactor(light_last[-1], eye_last[-1]));
  }

  if (std::distance(eye_first, eye_last) >= 2) {
    std::transform(
      std::make_reverse_iterator(eye_last),
      std::make_reverse_iterator(eye_first + 1),
      std::back_inserter(geometry_factor_),
      [](const auto& event){ return event.GeometryFactor(); }
    );
  }
}

template <typename Radiant>
void
BidirectionalPathSamplingBuffer<Radiant>
::BufferLightProbability(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  SubpathConstIterator light_first,
  SubpathConstIterator light_last,
  SubpathConstIterator eye_first,
  SubpathConstIterator eye_last
)
{
  p_light_.clear();

  if (std::distance(light_first, light_last) >= 2) {
    std::transform(
      light_first + 1,
      light_last,
      std::back_inserter(p_light_),
      [](const auto& event){ return event.BackwardPDF(); }
    );
  }

  if (light_first != light_last && eye_first != eye_last) {
    const auto& light_end = light_last[-1];
    const auto& eye_end   = eye_last[-1];

    const auto object        = eye_end.Object();
    const auto normal        = eye_end.Normal();
    const auto direction_out = eye_end.DirectionOut();
    const auto direction_in  =
      Normalize(light_end.Position() - eye_end.Position());

    const auto bsdf =
      scene.BSDF(object, normal, direction_out, direction_in);
    const auto pdf =
      scene.PDFLight(object, normal, direction_out, direction_in);
    const auto p_russian_roulette =
      std::min<real_type>(kRussianRoulette, Max(bsdf / pdf));

    p_light_.emplace_back(pdf * p_russian_roulette);
  }

  if (std::distance(eye_first, eye_last) >= 2) {
    std::transform(
      std::make_reverse_iterator(eye_last - 1),
      std::make_reverse_iterator(eye_first),
      std::back_inserter(p_light_),
      [](const auto& event){ return event.ForwardPDF(); }
    );
  }

  const auto z0 = eye_first == eye_last ? --light_last : eye_first++;
  const auto z1 = eye_first == eye_last ? --light_last : eye_first;
  p_light_.back() = scene.EyePDFDirection(
    sensor,
    Ray(z0->Position(), Normalize(z1->Position() - z0->Position()))
  );
}

template <typename Radiant>
void
BidirectionalPathSamplingBuffer<Radiant>
::BufferImportanceProbability(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  SubpathConstIterator light_first,
  SubpathConstIterator light_last,
  SubpathConstIterator eye_first,
  SubpathConstIterator eye_last
)
{
  p_importance_.clear();

  if (std::distance(light_first, light_last) >= 2) {
    std::transform(
      light_first,
      light_last - 1,
      std::back_inserter(p_importance_),
      [](const auto& event){ return event.ForwardPDF(); }
    );
  }

  if (light_first != light_last && eye_first != eye_last) {
    const auto& light_end = light_last[-1];
    const auto& eye_end   = eye_last[-1];

    const auto object        = light_end.Object();
    const auto normal        = light_end.Normal();
    const auto direction_out = light_end.DirectionOut();
    const auto direction_in  =
      Normalize(eye_end.Position() - light_end.Position());

    const auto bsdf =
      scene.AdjointBSDF(object, normal, direction_out, direction_in);
    const auto pdf =
      scene.PDFImportance(object, normal, direction_out, direction_in);
    const auto p_russian_roulette =
      std::min<real_type>(kRussianRoulette, Max(bsdf / pdf));

    p_importance_.emplace_back(pdf * p_russian_roulette);
  }

  if (std::distance(eye_first, eye_last) >= 2) {
    std::transform(
      std::make_reverse_iterator(eye_last),
      std::make_reverse_iterator(eye_first + 1),
      std::back_inserter(p_importance_),
      [](const auto& event){ return event.BackwardPDF(); }
    );
  }

  const auto y0 = light_first == light_last ? --eye_last : light_first++;
  const auto y1 = light_first == light_last ? --eye_last : light_first;
  p_importance_.front() = scene.LightPDFDirection(
    y0->Object(),
    Ray(y0->Position(), Normalize(y1->Position() - y0->Position()))
  );
}

template <typename Radiant>
void
BidirectionalPathSamplingBuffer<Radiant>
::BufferTechniqueProbability(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  SubpathConstIterator light_first,
  SubpathConstIterator light_last,
  SubpathConstIterator eye_first,
  SubpathConstIterator eye_last
)
{
  const auto s = std::distance(light_first, light_last);
  const auto t = std::distance(eye_first, eye_last);
  const auto n_techniques = s + t + 1;

  p_technique_.clear();
  p_technique_.resize(n_techniques, 1);

  for (path_size_type i = s - 1; i < n_techniques; i--) {
    auto& p = p_technique_[i];
    p = p_technique_[i + 1];

    if (t == 0 && i == n_techniques - 2) {
      p *= scene.EyePDFArea(light_last[-1].Position());
    } else {
      p *= p_light_[i] * geometry_factor_[i];
    }

    if (s > 0 && i == 0) {
      p /= scene.LightPDFArea(light_first->Object());
    } else {
      p /= p_importance_[i - 1] * geometry_factor_[i - 1];
    }
  }

  for (path_size_type i = s + 1; i < n_techniques; i++) {
    auto& p = p_technique_[i];
    p = p_technique_[i - 1];

    if (s == 0 && i == 1) {
      p *= scene.LightPDFArea(eye_last[-1].Object());
    } else {
      p *= p_importance_[i - 2] * geometry_factor_[i - 2];
    }

    if (t > 0 && i == n_techniques - 1) {
      p /= scene.EyePDFArea(eye_first->Position());
    } else {
      p /= p_light_[i - 1] * geometry_factor_[i - 1];
    }
  }
}

template <typename Radiant>
void
BidirectionalPathSamplingBuffer<Radiant>
::DisableSpecularConnection(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  SubpathConstIterator light_first,
  SubpathConstIterator light_last,
  SubpathConstIterator eye_first,
  SubpathConstIterator eye_last
)
{
  const auto s = std::distance(light_first, light_last);
  const auto t = std::distance(eye_first, eye_last);
  const auto n_techniques = s + t + 1;

  for (std::size_t i = 1; i + 1 < s; i++) {
    if (scene.Surface(light_first[i].Object()) != SurfaceType::Diffuse) {
      p_technique_[i] = 0;
      p_technique_[i + 1] = 0;
    }
  }

  for (std::size_t i = 1; i + 1 < t; i++) {
    if (scene.Surface(eye_first[i].Object()) != SurfaceType::Diffuse) {
      p_technique_[n_techniques - i - 1] = 0;
      p_technique_[n_techniques - i - 2] = 0;
    }
  }
}

template <typename Radiant>
const real_type
GeometryFactor(
  const SubpathEvent<Radiant>& x,
  const SubpathEvent<Radiant>& y
) noexcept
{
  return
    prelude::GeometryFactor(x.Position(), y.Position(), x.Normal(), y.Normal());
}

template <typename Radiant>
void
GenerateLightSubpath(
  const Scene<Radiant>& scene,
  Sampler& sampler,
  typename Subpath<Radiant>::BackInsertIterator inserter
)
{
  GenerateLightSubpath(
    scene,
    sampler,
    inserter,
    std::numeric_limits<path_size_type>::max()
  );
}

template <typename Radiant>
void
GenerateLightSubpath(
  const Scene<Radiant>& scene,
  Sampler& sampler,
  typename Subpath<Radiant>::BackInsertIterator inserter,
  path_size_type s
)
{
  if (s == 0) {
    return;
  }

  auto generated = scene.GenerateLightRay(sampler);
  auto& object  = std::get<ObjectPointer>(generated);
  auto& leading = std::get<Leading<Radiant>>(generated);

  inserter = SubpathEvent<Radiant>(object, leading);

  auto ray    = leading.Ray();
  auto normal = leading.Normal();
  auto weight = leading.Weight();

  for (path_size_type i = 1; i < s; i++) {
    Hit hit;
    std::tie(object, hit) = scene.Cast(ray);
    if (!hit) {
      break;
    }

    const auto direction_out = -ray.Direction();
    const auto scatter =
      scene.SampleImportance(object, hit.Normal(), direction_out, sampler);
    const auto direction_in = scatter.DirectionIn();

    // forward transport = importance transport (in this context)
    const auto p_forward =
      scene.PDFImportance(object, hit.Normal(), direction_out, direction_in);
    const auto p_russian_roulette_forward =
      std::min<real_type>(kRussianRoulette, Max(scatter.Weight()));

    // backward transport = light transport (in this context)
    const auto p_backward =
      scene.PDFLight(object, hit.Normal(), direction_in, direction_out);
    const auto bsdf_backward =
      scene.BSDF(object, hit.Normal(), direction_in, direction_out);
    const auto p_russian_roulette_backward =
      std::min<real_type>(kRussianRoulette, Max(bsdf_backward / p_backward));

    const auto geometry_factor = prelude::GeometryFactor(
      direction_out,
      hit.Distance() * hit.Distance(),
      normal,
      hit.Normal()
    );

    inserter = SubpathEvent<Radiant>(
      object,
      hit.Position(),
      hit.Normal(),
      direction_out,
      weight,
      geometry_factor,
      p_backward * p_russian_roulette_backward,
      p_forward * p_russian_roulette_forward
    );

    if (prelude::Uniform<real_type>(sampler) >= p_russian_roulette_forward) {
      break;
    }

    ray = Ray(hit.Position(), direction_in);
    normal = hit.Normal();
    weight *= scatter.Weight() / p_russian_roulette_forward;
  }
}

template <typename Radiant>
Pixel
GenerateEyeSubpath(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  Sampler& sampler,
  typename Subpath<Radiant>::BackInsertIterator inserter
)
{
  return GenerateEyeSubpath(
    scene,
    sensor,
    sampler,
    inserter,
    std::numeric_limits<path_size_type>::max()
  );
}

template <typename Radiant>
Pixel
GenerateEyeSubpath(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  Sampler& sampler,
  typename Subpath<Radiant>::BackInsertIterator inserter,
  path_size_type t
)
{
  if (t == 0) {
    return Pixel();
  }

  auto generated = scene.GenerateEyeRay(sensor, sampler);
  auto& object  = std::get<ObjectPointer>(generated);
  auto& leading = std::get<Leading<Radiant>>(generated);

  inserter = SubpathEvent<Radiant>(object, leading);

  auto ray    = leading.Ray();
  auto normal = leading.Normal();
  auto weight = leading.Weight();

  for (path_size_type i = 1; i < t; i++) {
    Hit hit;
    std::tie(object, hit) = scene.Cast(ray);
    if (!hit) {
      break;
    }

    const auto direction_out = -ray.Direction();
    const auto scatter =
      scene.SampleLight(object, hit.Normal(), direction_out, sampler);
    const auto direction_in = scatter.DirectionIn();

    // forward transport = light transport (in this context)
    const auto p_forward =
      scene.PDFLight(object, hit.Normal(), direction_out, direction_in);
    const auto p_russian_roulette_forward =
      std::min<real_type>(kRussianRoulette, Max(scatter.Weight()));

    // backward transport = importance transport (in this context)
    const auto p_backward =
      scene.PDFImportance(object, hit.Normal(), direction_in, direction_out);
    const auto bsdf_backward =
      scene.AdjointBSDF(object, hit.Normal(), direction_in, direction_out);
    const auto p_russian_roulette_backward =
      std::min<real_type>(kRussianRoulette, Max(bsdf_backward / p_backward));

    const auto geometry_factor = prelude::GeometryFactor(
      direction_out,
      hit.Distance() * hit.Distance(),
      normal,
      hit.Normal()
    );

    inserter = SubpathEvent<Radiant>(
      object,
      hit.Position(),
      hit.Normal(),
      direction_out,
      weight,
      geometry_factor,
      p_backward * p_russian_roulette_backward,
      p_forward * p_russian_roulette_forward
    );

    if (prelude::Uniform<real_type>(sampler) >= p_russian_roulette_forward) {
      break;
    }

    ray = Ray(hit.Position(), direction_in);
    normal = hit.Normal();
    weight *= scatter.Weight() / p_russian_roulette_forward;
  }

  return std::get<Pixel>(generated);
}

template <typename Radiant>
PixelValue<Radiant>
Connect(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  const SubpathEvent<Radiant>* light_endpoint,
  const SubpathEvent<Radiant>* eye_endpoint
)
{
  if (light_endpoint == nullptr && eye_endpoint == nullptr) {
    return Radiant();
  }

  if (light_endpoint == nullptr) {
    // s = 0; the eye subpath intersects a light source
    if (scene.Surface(eye_endpoint->Object()) != SurfaceType::Light) {
      return Radiant();
    }
    return
      eye_endpoint->Weight() *
      scene.Radiance(
        eye_endpoint->Object(),
        eye_endpoint->Normal(),
        eye_endpoint->DirectionOut()
      );
  }

  if (eye_endpoint == nullptr) {
    // t = 0; the light subpath intersects the aperture
    if (scene.Surface(light_endpoint->Object()) != SurfaceType::Eye) {
      return Radiant();
    }
    return
      light_endpoint->Weight() *
      scene.Response(
        sensor,
        light_endpoint->Position(),
        light_endpoint->DirectionOut()
      );
  }

  // after this, we need to connect a light and an eye subpath.

  if (!light_endpoint->IsLeading() &&
      scene.Surface(light_endpoint->Object()) != SurfaceType::Diffuse) {
    return Radiant();
  }

  if (!eye_endpoint->IsLeading() &&
      scene.Surface(eye_endpoint->Object()) != SurfaceType::Diffuse) {
    return Radiant();
  }

  PixelValue<Radiant> contribution(1);

  const auto shadow_ray = Ray(
    light_endpoint->Position(),
    eye_endpoint->Position() - light_endpoint->Position()
  );

  if (light_endpoint->IsLeading()) {
    // s = 1
    contribution *=
      scene.Radiance(
        light_endpoint->Object(),
        light_endpoint->Normal(),
        shadow_ray.Direction()
      ) /
      scene.LightPDFArea(light_endpoint->Object());
  } else {
    // s > 1
    contribution *=
      light_endpoint->Weight() *
      scene.AdjointBSDF(
        light_endpoint->Object(),
        light_endpoint->Normal(),
        light_endpoint->DirectionOut(),
        shadow_ray.Direction()
      );
  }

  if (eye_endpoint->IsLeading()) {
    // t = 1
    contribution =
      contribution.Value() *
      scene.Response(
        sensor,
        eye_endpoint->Position(),
        -shadow_ray.Direction()
      ) /
      static_cast<Radiant>(scene.EyePDFArea(eye_endpoint->Position()));
  } else {
    // t > 1
    contribution *=
      eye_endpoint->Weight() *
      scene.BSDF(
        eye_endpoint->Object(),
        eye_endpoint->Normal(),
        eye_endpoint->DirectionOut(),
        -shadow_ray.Direction()
      );
  }

  if (!contribution) {
    return Radiant();
  }

  // XXX rough check
  const auto object = std::get<ObjectPointer>(scene.Cast(shadow_ray));
  if (object != eye_endpoint->Object()) {
    return Radiant();
  }

  contribution *= GeometryFactor(*light_endpoint, *eye_endpoint);

  return contribution;
}

template <typename Radiant>
const Radiant
Combine(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  const Subpath<Radiant>& light_path,
  const Subpath<Radiant>& eye_path,
  const MIS& mis,
  BidirectionalPathSamplingBuffer<Radiant>& buffer,
  SparseImage<Radiant>& light_image
)
{
  Radiant measurement(0);

  for (path_size_type s = 0; s <= light_path.Size(); s++) {
    for (path_size_type t = 0; t <= eye_path.Size(); t++) {
      auto contribution = Connect(
        scene,
        sensor,
        s == 0 ? nullptr : &light_path.begin()[s - 1],
        t == 0 ? nullptr : &eye_path.begin()[t - 1]
      );

      if (!contribution) {
        continue;
      }

      buffer.Buffer(
        scene,
        sensor,
        light_path.begin(),
        light_path.begin() + s,
        eye_path.begin(),
        eye_path.begin() + t
      );

      contribution *= buffer.MISWeight(mis);

      if (contribution.Pixel()) {
        // light_image
        light_image.Emplace(contribution / static_cast<Radiant>(sensor.Size()));
      } else {
        // eye_image
        measurement += contribution.Value();
      }
    }
  }

  return measurement;
}

}
}
