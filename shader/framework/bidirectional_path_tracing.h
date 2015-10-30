/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

#include "base/constant.h"
#include "base/sampler.h"
#include "material/surface_type.h"
#include "shader/framework/light_sampler.h"

namespace amber {
namespace shader {
namespace framework {

template <typename Acceleration,
          typename Object = typename Acceleration::object_type>
class BidirectionalPathTracing {
private:
  using acceleration_ptr   = std::shared_ptr<Acceleration>;
  using light_sampler_ptr  = std::shared_ptr<LightSampler<Object>>;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_type::value_type;
  using ray_type           = typename Object::ray_type;
  using vector3_type       = typename Object::vector3_type;

  struct Event {
    Object object;
    vector3_type position;
    vector3_type normal;
    vector3_type direction_i;
    vector3_type direction_o;
    radiant_value_type p_russian_roulette;
    radiant_value_type log_p_area;
    radiant_type weight;
  };

  acceleration_ptr acceleration_;
  light_sampler_ptr light_sampler_;

public:
  BidirectionalPathTracing(acceleration_ptr const& acceleration,
                           light_sampler_ptr const& light_sampler) noexcept
   : acceleration_(acceleration), light_sampler_(light_sampler) {}

  std::vector<Event> lightPathTracing(Sampler* sampler) const {
    auto const light = (*light_sampler_)(sampler);
    auto const ray = light.sampleFirstRay(sampler);
    auto const p_area = lightPDFArea(light);
    auto const p_psa = static_cast<radiant_value_type>(1 / kPI);

    Event event;
    event.object             = light;
    event.position           = ray.origin;
    event.normal             = ray.normal;
    event.direction_i        = vector3_type();
    event.direction_o        = ray.direction;
    event.p_russian_roulette = 1;
    event.log_p_area         = std::log2(p_area);
    event.weight             = light.emittance() * kPI / p_area;

    return pathTracing(event, p_psa, sampler);
  }

  template <typename Camera>
  std::vector<Event> eyePathTracing(Sampler* sampler,
                                    Camera const& camera,
                                    size_t x,
                                    size_t y) const {
    auto const importance = radiant_type(kDiracDelta); // TODO make eye diffuse
    auto const ray = camera.sampleFirstRay(x, y, sampler);
    auto const p_area = eyePDFArea();
    auto const p_psa = static_cast<radiant_value_type>(1 / kPI);

    Event event;
    event.object             = Object();
    event.position           = ray.origin;
    event.normal             = ray.normal;
    event.direction_i        = vector3_type();
    event.direction_o        = ray.direction;
    event.p_russian_roulette = 1;
    event.log_p_area         = std::log2(p_area);
    event.weight             = importance / p_area;

    return pathTracing(event, p_psa, sampler);
  }

  template <typename MIS>
  radiant_type connect(std::vector<Event> const& light,
                       std::vector<Event> const& eye,
                       MIS const& mis) const {
    radiant_type power;

    for (size_t s = 0; s <= light.size(); s++) {
      for (size_t t = 0; t <= eye.size(); t++) {
        auto const contribution = unweightedContribution(light, eye, s, t);
        if (contribution.max() == 0) {
          continue;
        }
        auto const log_ps = misLogProbabilities(light, eye, s, t);
        auto const weight = mis(log_ps.begin(), log_ps.end(), log_ps.at(s));
        power += contribution * weight;
      }
    }

    return power;
  }

private:
  radiant_value_type lightPDFArea(Object const& light) const noexcept {
    return light.emittance().sum() * kPI / light_sampler_->total_power().sum();
  }

  radiant_value_type eyePDFArea() const noexcept {
    return kDiracDelta; // TODO make eye diffuse
  }

  radiant_value_type
  geometryFactor(Event const& x,
                 Event const& y) const noexcept {
    return geometryFactor(x, y, normalize(y.position - x.position));
  }

  radiant_value_type
  geometryFactor(Event const& x,
                 Event const& y,
                 vector3_type const& direction) const noexcept {
    return std::abs(dot(direction, x.normal) *
                    dot(direction, y.normal) /
                    (y.position - x.position).squaredLength());
  }

  std::vector<Event> pathTracing(Event event,
                                 radiant_value_type p_psa,
                                 Sampler* sampler) const {
    std::vector<Event> events;
    events.reserve(16);
    events.push_back(event);

    ray_type ray(event.position, event.direction_o);
    radiant_type bsdf(p_psa);
    hit_type hit;
    Object object;

    for (;;) {
      std::tie(hit, object) = acceleration_->cast(ray);
      if (!hit) {
        break;
      }

      auto const sample =
        object.sampleScatter(-ray.direction, hit.normal, sampler);
      auto const geometry_factor =
        std::abs(dot(ray.direction, event.normal) *
                 dot(ray.direction, hit.normal)) /
        (hit.distance * hit.distance);
      auto const p_russian_roulette =
        std::min(static_cast<radiant_value_type>(1),
                 (sample.bsdf / sample.psa_probability).max());

      event.object              = object;
      event.position            = hit.position;
      event.normal              = hit.normal;
      event.direction_i         = -ray.direction;
      event.direction_o         = sample.direction_o;
      event.p_russian_roulette  = p_russian_roulette;
      event.log_p_area         += std::log2(p_psa * geometry_factor);
      event.weight             *= bsdf / p_psa;
      events.push_back(event);

      ray = ray_type(hit.position, sample.direction_o);
      bsdf = sample.bsdf;
      p_psa = sample.psa_probability * p_russian_roulette;
      if (sampler->uniform<radiant_value_type>() >= p_russian_roulette) {
        break;
      }
    }

    return events;
  }

  radiant_type unweightedContribution(std::vector<Event> const& light,
                                      std::vector<Event> const& eye,
                                      size_t s,
                                      size_t t) const {
    if (s >= 2 && t >= 2) {
      auto const& l = light.at(s - 1);
      auto const& e = eye.at(t - 1);
      if (l.object.surfaceType() == material::SurfaceType::specular) {
        return radiant_type();
      }
      if (e.object.surfaceType() == material::SurfaceType::specular) {
        return radiant_type();
      }
      ray_type const ray(l.position, e.position - l.position);
      if (!acceleration_->test_visibility(ray, e.object)) {
        return radiant_type();
      }
      auto const direction_le = normalize(e.position - l.position);
      return
        l.weight *
        l.object.bsdf(l.direction_i, direction_le, l.normal) *
        geometryFactor(l, e, direction_le) *
        e.object.bsdf(-direction_le, e.direction_o, e.normal) *
        e.weight;
    } else if (s == 0 && t >= 2) {
      // zero light subpath vertices
      auto const& l = eye.at(t - 1);
      auto const& e = eye.at(t - 2);
      if (!l.object.isEmissive()) {
        return radiant_type();
      }
      if (dot(e.position - l.position, l.normal) <= 0) {
        return radiant_type();
      }
      return l.object.emittance() * l.weight;
    } else if (s == 1 && t >= 2) {
      // one light subpath vertex
      auto const& l = light.at(s - 1);
      auto const& e = eye.at(t - 1);
      if (e.object.surfaceType() == material::SurfaceType::specular) {
        return radiant_type();
      }
      if (dot(e.position - l.position, l.normal) <= 0) {
        return radiant_type();
      }
      ray_type const ray(l.position, e.position - l.position);
      if (!acceleration_->test_visibility(ray, e.object)) {
        return radiant_type();
      }
      auto const direction_le = normalize(e.position - l.position);
      return
        l.weight *
        (1 / kPI) *
        geometryFactor(l, e, direction_le) *
        e.object.bsdf(-direction_le, e.direction_o, e.normal) *
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

  std::vector<radiant_value_type>
  misLogProbabilities(std::vector<Event> const& light,
                      std::vector<Event> const& eye,
                      size_t s,
                      size_t t) const {
    std::vector<Event> events;
    std::copy(light.begin(), light.begin() + s, std::back_inserter(events));
    std::transform(eye.rend() - t, eye.rend(), std::back_inserter(events),
      [](auto event){
        std::swap(event.direction_i, event.direction_o);
        return event;
      });

    if (s > 0 && t > 0) {
      auto const direction =
        normalize(events.at(s).position - events.at(s - 1).position);
      events.at(s - 1).direction_o = direction;
      events.at(s).direction_i = direction;
    }

    size_t const n_technique = s + t + 1;
    std::vector<radiant_value_type> ps(n_technique);
    for (size_t i = 1; s > 0 && i < n_technique; i++) {
      ps.at(i) += events.at(std::min(i, s) - 1).log_p_area;
    }
    for (size_t i = 0; t > 0 && i < n_technique - 1; i++) {
      ps.at(i) += events.at(std::max(i, s)).log_p_area;
    }
    {
      // extend light subpath
      radiant_value_type log_p_light = 0;
      auto p_russian_roulette =
        s == 0 ? 1 : events.at(s - 1).p_russian_roulette;
      for (size_t i = s + 1; i < n_technique; i++) {
        if (s == 0) {
          log_p_light += std::log2(lightPDFArea(events.front().object));
        } else {
          auto& event = events.at(i - 2);
          auto const bsdf = event.object.bsdf(event.direction_i,
                                              event.direction_o,
                                              event.normal);
          auto const pdf = event.object.pdf(event.direction_i,
                                            event.direction_o,
                                            event.normal);
          auto const geometry_factor =
            geometryFactor(event, events.at(i - 1));
          log_p_light +=
            std::log2(pdf * p_russian_roulette * geometry_factor);
          p_russian_roulette =
            std::min<radiant_value_type>(1, (bsdf / pdf).max());
        }
        ps.at(i) += log_p_light;
      }
    }
    {
      // extend eye subpath
      radiant_value_type log_p_eye = 0;
      auto p_russian_roulette =
        t == 0 ? 1 : events.at(s).p_russian_roulette;
      for (size_t i = s - 1; i < n_technique; i--) {
        if (t == 0) {
          log_p_eye += std::log2(eyePDFArea());
        } else {
          auto& event = events.at(i + 1);
          auto const bsdf = event.object.bsdf(event.direction_o,
                                              event.direction_i,
                                              event.normal);
          auto const pdf = event.object.pdf(event.direction_o,
                                            event.direction_i,
                                            event.normal);
          auto const geometry_factor =
            geometryFactor(event, events.at(i));
          log_p_eye +=
            std::log2(pdf * p_russian_roulette * geometry_factor);
          p_russian_roulette =
            std::min<radiant_value_type>(1, (bsdf / pdf).max());
        }
        ps.at(i) += log_p_eye;
      }
    }
    {
      // ignore contributions by a connection with specular surfaces
      for (size_t i = 0; i < s + t; i++) {
        auto const& object = events.at(i).object;
        if (object &&
            object.surfaceType() == material::SurfaceType::diffuse) {
          continue;
        }
        ps.at(i)     = -std::numeric_limits<radiant_value_type>::infinity();
        ps.at(i + 1) = -std::numeric_limits<radiant_value_type>::infinity();
      }
    }

    return ps;
  }
};

}
}
}
