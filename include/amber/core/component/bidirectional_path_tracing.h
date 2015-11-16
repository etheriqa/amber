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
#include <cmath>
#include <vector>

#include "core/constant.h"
#include "core/geometry.h"
#include "core/sampler.h"
#include "core/scene.h"
#include "core/surface_type.h"

namespace amber {
namespace core {
namespace component {

template <typename Object>
class BidirectionalPathTracing
{
public:
  using scene_type         = Scene<Object>;

private:
  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_type::value_type;
  using ray_type           = typename Object::ray_type;
  using real_type          = typename Object::real_type;
  using vector3_type       = typename Object::vector3_type;

  struct Event
  {
    Object object;
    vector3_type position;
    vector3_type normal;
    vector3_type direction_i;
    vector3_type direction_o;
    radiant_value_type p_russian_roulette;
    radiant_value_type log_p_area;
    radiant_type weight;
  };

  scene_type scene_;
  std::vector<Event> mutable event_buffer_;
  std::vector<radiant_value_type> mutable probability_buffer_;

public:
  explicit BidirectionalPathTracing(scene_type const& scene) noexcept
  : scene_(scene),
    event_buffer_(),
    probability_buffer_()
  {}

  std::vector<Event> lightTracing(
    Sampler& sampler
  ) const
  {
    ray_type ray;
    Object light;
    radiant_type weight;
    radiant_value_type p_area;
    radiant_value_type p_psa = 1 / kPI;
    vector3_type normal;
    std::tie(ray, weight, light, p_area, normal) =
      scene_.GenerateLightRay(sampler);

    Event event;
    event.object             = light;
    event.position           = ray.origin;
    event.normal             = normal;
    event.direction_i        = ray.direction;
    event.direction_o        = vector3_type();
    event.p_russian_roulette = 1;
    event.log_p_area         = std::log2(p_area);
    event.weight             = weight;

    return tracing(
      event, p_psa, sampler,
      [](
        auto const& object,
        auto const& direction_o,
        auto const& normal,
        auto& sampler
      )
      {
        return object.SampleImportance(direction_o, normal, sampler);
      },
      [](
        auto const& object,
        auto const& direction_i,
        auto const& direction_o,
        auto const& normal
      )
      {
        return object.PDFImportance(direction_i, direction_o, normal);
      }
    );
  }

  template <typename Camera>
  std::vector<Event> rayTracing(
    Sampler& sampler,
    Camera const& camera,
    size_t x,
    size_t y
  ) const
  {
    ray_type ray;
    radiant_type weight;
    radiant_value_type p_area;
    radiant_value_type p_psa = 1;
    vector3_type normal;
    std::tie(ray, weight, p_area, normal) = camera.GenerateRay(x, y, sampler);

    Event event;
    event.object             = Object();
    event.position           = ray.origin;
    event.normal             = normal;
    event.direction_i        = ray.direction;
    event.direction_o        = vector3_type();
    event.p_russian_roulette = 1;
    event.log_p_area         = std::log2(p_area);
    event.weight             = weight;

    return tracing(
      event, p_psa, sampler,
      [](
        auto const& object,
        auto const& direction_o,
        auto const& normal,
        auto& sampler
      )
      {
        return object.SampleLight(direction_o, normal, sampler);
      },
      [](
        auto const& object,
        auto const& direction_i,
        auto const& direction_o,
        auto const& normal
      )
      {
        return object.PDFLight(direction_i, direction_o, normal);
      }
    );
  }

  template <typename MIS>
  radiant_type connect(
    std::vector<Event> const& light,
    std::vector<Event> const& eye,
    MIS const& mis
  ) const
  {
    radiant_type power;

    for (size_t s = 0; s <= light.size(); s++) {
      for (size_t t = 0; t <= eye.size(); t++) {
        auto const contribution = UnweightedContribution(light, eye, s, t);
        if (Max(contribution) == 0) {
          continue;
        }
        CalculateLogProbabilities(light, eye, s, t);
        auto const weight = mis(
          probability_buffer_.begin(),
          probability_buffer_.end(),
          probability_buffer_.at(s)
        );
        power += contribution * weight;
      }
    }

    return power;
  }

private:
  real_type
  GeometryFactor(
    Event const& x,
    Event const& y
  ) const noexcept
  {
    return core::GeometryFactor(x.position, y.position, x.normal, y.normal);
  }

  template <typename Sample, typename PDF>
  std::vector<Event> tracing(
    Event event,
    radiant_value_type p_psa,
    Sampler& sampler,
    Sample const& sample,
    PDF const& pdf
  ) const
  {
    std::vector<Event> events;
    events.reserve(16);
    events.push_back(event);

    ray_type ray(event.position, event.direction_i);
    radiant_type weight(1);
    hit_type hit;
    Object object;

    for (;;) {
      std::tie(hit, object) = scene_.Cast(ray);
      if (!hit) {
        break;
      }

      auto const scatter = sample(object, -ray.direction, hit.normal, sampler);
      auto const geometry_factor = core::GeometryFactor(
        ray.direction,
        hit.distance * hit.distance,
        event.normal,
        hit.normal
      );
      auto const p_russian_roulette =
        std::min<radiant_value_type>(1, Max(scatter.weight));

      event.object              = object;
      event.position            = hit.position;
      event.normal              = hit.normal;
      event.direction_i         = scatter.direction;
      event.direction_o         = -ray.direction;
      event.p_russian_roulette  = p_russian_roulette;
      event.log_p_area         += std::log2(p_psa * geometry_factor);
      event.weight             *= weight;
      events.push_back(event);

      if (Uniform<radiant_value_type>(sampler) >= p_russian_roulette) {
        break;
      }

      ray = ray_type(hit.position, scatter.direction);
      weight = scatter.weight / p_russian_roulette;
      p_psa =
        pdf(event.object, event.direction_i, event.direction_o, event.normal) *
        p_russian_roulette;
    }

    return events;
  }

  radiant_type UnweightedContribution(
    std::vector<Event> const& light,
    std::vector<Event> const& eye,
    size_t s,
    size_t t
  ) const
  {
    if (s >= 2 && t >= 2) {
      auto const& l = light.at(s - 1);
      auto const& e = eye.at(t - 1);
      if (l.object.Surface() == SurfaceType::Specular) {
        return radiant_type();
      }
      if (e.object.Surface() == SurfaceType::Specular) {
        return radiant_type();
      }
      ray_type const ray(l.position, e.position - l.position);
      if (!scene_.TestVisibility(ray, e.object)) {
        return radiant_type();
      }
      auto const direction_le = Normalize(e.position - l.position);
      return
        l.weight *
        l.object.BSDF(l.direction_o, direction_le, l.normal) *
        GeometryFactor(l, e) *
        e.object.BSDF(-direction_le, e.direction_o, e.normal) *
        e.weight;
    } else if (s == 0 && t >= 2) {
      // zero light subpath vertices
      auto const& l = eye.at(t - 1);
      auto const& e = eye.at(t - 2);
      if (l.object.Surface() != SurfaceType::Light) {
        return radiant_type();
      }
      if (Dot(e.position - l.position, l.normal) <= 0) {
        return radiant_type();
      }
      return l.object.Radiance() * l.weight;
    } else if (s == 1 && t >= 2) {
      // one light subpath vertex
      auto const& l = light.at(s - 1);
      auto const& e = eye.at(t - 1);
      if (e.object.Surface() == SurfaceType::Specular) {
        return radiant_type();
      }
      if (Dot(e.position - l.position, l.normal) <= 0) {
        return radiant_type();
      }
      ray_type const ray(l.position, e.position - l.position);
      if (!scene_.TestVisibility(ray, e.object)) {
        return radiant_type();
      }
      auto const direction_le = Normalize(e.position - l.position);
      return
        l.weight *
        (1 / kPI) *
        GeometryFactor(l, e) *
        e.object.BSDF(-direction_le, e.direction_o, e.normal) *
        e.weight;
    } else if (s >= 2 && t == 0) {
      // zero eye subpath vertices
      // TODO make eye diffuse
    } else if (s >= 2 && t == 1) {
      // one eye subpath vertex
      // TODO make eye diffuse
    } else if (s == 1 && t == 1) {
      // one light subpath vertex and one eye subpath vertex
      // TODO make eye diffuse
    }

    return radiant_type();
  }

  // caution: this function is not thread-safe
  void CalculateLogProbabilities(
    std::vector<Event> const& light,
    std::vector<Event> const& eye,
    size_t s,
    size_t t
  ) const
  {
    event_buffer_.clear();
    probability_buffer_.clear();

    // reorder pairs of directions (direction_i and direction_o) by actual light
    // flow
    std::transform(
      light.begin(),
      light.begin() + s,
      std::back_inserter(event_buffer_),
      [](auto event){
        std::swap(event.direction_i, event.direction_o);
        return event;
      });
    std::copy(eye.rend() - t, eye.rend(), std::back_inserter(event_buffer_));

    if (s > 0 && t > 0) {
      auto const direction = Normalize(
        event_buffer_.at(s).position - event_buffer_.at(s - 1).position
      );
      event_buffer_.at(s - 1).direction_o = direction;
      event_buffer_.at(s).direction_i = direction;
    }

    size_t const n_technique = s + t + 1;
    probability_buffer_.resize(n_technique);
    for (size_t i = 1; s > 0 && i < n_technique; i++) {
      probability_buffer_.at(i) +=
        event_buffer_.at(std::min(i, s) - 1).log_p_area;
    }
    for (size_t i = 0; t > 0 && i < n_technique - 1; i++) {
      probability_buffer_.at(i) +=
        event_buffer_.at(std::max(i, s)).log_p_area;
    }
    {
      // extend light subpath
      radiant_value_type log_p_light = 0;
      auto p_russian_roulette =
        s == 0 ? 1 : event_buffer_.at(s - 1).p_russian_roulette;
      for (size_t i = s + 1; i < n_technique; i++) {
        if (s == 0) {
          log_p_light +=
            std::log2(scene_.LightPDFArea(event_buffer_.front().object));
        } else {
          auto& event = event_buffer_.at(i - 2);
          auto const bsdf = event.object.BSDF(
            event.direction_i,
            event.direction_o,
            event.normal
          );
          auto const pdf = event.object.PDFImportance(
            event.direction_o,
            event.direction_i,
            event.normal
          );
          auto const geometry_factor =
            GeometryFactor(event, event_buffer_.at(i - 1));
          log_p_light +=
            std::log2(pdf * p_russian_roulette * geometry_factor);
          p_russian_roulette =
            std::min<radiant_value_type>(1, Max(bsdf / pdf));
        }
        probability_buffer_.at(i) += log_p_light;
      }
    }
    {
      // extend eye subpath
      radiant_value_type log_p_eye = 0;
      auto p_russian_roulette =
        t == 0 ? 1 : event_buffer_.at(s).p_russian_roulette;
      for (size_t i = s - 1; i < n_technique; i--) {
        if (t == 0) {
          log_p_eye += std::log2(kDiracDelta); // TODO
        } else {
          auto& event = event_buffer_.at(i + 1);
          auto const bsdf = event.object.BSDF(
            event.direction_i,
            event.direction_o,
            event.normal
          );
          auto const pdf = event.object.PDFLight(
            event.direction_i,
            event.direction_o,
            event.normal
          );
          auto const geometry_factor =
            GeometryFactor(event, event_buffer_.at(i));
          log_p_eye +=
            std::log2(pdf * p_russian_roulette * geometry_factor);
          p_russian_roulette =
            std::min<radiant_value_type>(1, Max(bsdf / pdf));
        }
        probability_buffer_.at(i) += log_p_eye;
      }
    }
    {
      // ignore contributions by a connection with specular surfaces
      for (size_t i = 0; i < s + t; i++) {
        auto const& object = event_buffer_.at(i).object;
        if (object && object.Surface() != SurfaceType::Specular) {
          continue;
        }
        probability_buffer_.at(i) =
          -std::numeric_limits<radiant_value_type>::infinity();
        probability_buffer_.at(i + 1) =
          -std::numeric_limits<radiant_value_type>::infinity();
      }
    }
  }
};

}
}
}
