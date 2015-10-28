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
    auto const area_pdf = lightAreaPDF(light);
    auto const psa_pdf = static_cast<radiant_value_type>(1 / kPI);

    Event event;
    event.object      = light;
    event.position    = ray.origin;
    event.normal      = ray.normal;
    event.direction_i = vector3_type();
    event.direction_o = ray.direction;
    event.weight      = light.emittance() * kPI / area_pdf;

    return pathTracing(event, psa_pdf, sampler);
  }

  template <typename Camera>
  std::vector<Event> eyePathTracing(Sampler* sampler,
                                    Camera const& camera,
                                    size_t x,
                                    size_t y) const {
    // TODO importance, area_pdf, psa_pdf
    auto const importance = radiant_type(1);
    auto const ray = camera.sampleFirstRay(x, y, sampler);
    auto const area_pdf = eyeAreaPDF();
    auto const psa_pdf = static_cast<radiant_value_type>(1 / kPI);

    Event event;
    event.object      = Object();
    event.position    = ray.origin;
    event.normal      = ray.normal;
    event.direction_i = vector3_type();
    event.direction_o = ray.direction;
    event.weight      = importance / area_pdf;

    auto events = pathTracing(event, psa_pdf, sampler);
    for (auto& event : events) {
      std::swap(event.direction_i, event.direction_o);
    }
    return events;
  }

  radiant_type connect(std::vector<Event> const& light,
                       std::vector<Event> const& eye) const {
    radiant_type power;

    for (size_t s = 0; s <= light.size(); s++) {
      for (size_t t = 0; t <= eye.size(); t++) {
        if (s + t < 2) {
          continue;
        }
        radiant_type unweighted_contribution;
        if (s >= 2 && t >= 2) {
          auto const& l = light.at(s - 1);
          auto const& e = eye.at(t - 1);
          if (l.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          if (e.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          ray_type const ray(l.position, e.position - l.position);
          if (!acceleration_->test_visibility(ray, e.object)) {
            continue;
          }
          auto const direction_le = normalize(e.position - l.position);
          unweighted_contribution =
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
            continue;
          }
          if (dot(e.position - l.position, l.normal) <= 0) {
            continue;
          }
          unweighted_contribution = l.object.emittance() * l.weight;
        } else if (s == 1 && t >= 2) {
          // one light subpath vertex
          auto const& l = light.at(s - 1);
          auto const& e = eye.at(t - 1);
          if (e.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          if (dot(e.position - l.position, l.normal) <= 0) {
            continue;
          }
          ray_type const ray(l.position, e.position - l.position);
          if (!acceleration_->test_visibility(ray, e.object)) {
            continue;
          }
          auto const direction_le = normalize(e.position - l.position);
          unweighted_contribution =
            l.weight *
              (1 / kPI) *
              geometryFactor(l, e, direction_le) *
              e.object.bsdf(-direction_le, e.direction_o, e.normal) *
              e.weight;
        } else if (s >= 2 && t == 0) {
          // TODO zero eye subpath vertices
          continue;
        } else if (s >= 2 && t == 1) {
          // TODO one eye subpath vertex
          continue;
        } else if (s == 1 && t == 1) {
          // TODO one light subpath vertex and one eye subpath vertex
          continue;
        }

        // TODO MIS
        power += unweighted_contribution / (s + t + 1);
      }
    }

    return power;
  }

private:
  radiant_value_type lightAreaPDF(Object const& light) const noexcept {
    return light.emittance().sum() * kPI / light_sampler_->total_power().sum();
  }

  radiant_value_type eyeAreaPDF() const noexcept {
    return 1;
  }

  std::vector<Event> pathTracing(Event event,
                                 radiant_value_type psa_pdf,
                                 Sampler* sampler) const {
    std::vector<Event> events;
    events.reserve(16);
    events.push_back(event);

    ray_type ray(event.position, event.direction_o);
    radiant_type bsdf(psa_pdf);
    hit_type hit;
    Object object;

    for (;;) {
      std::tie(hit, object) = acceleration_->cast(ray);
      if (!hit) {
        break;
      }

      auto const sample = object.sampleScatter(event.weight * bsdf,
                                               -ray.direction,
                                               hit.normal,
                                               sampler);

      event.object       = object;
      event.position     = hit.position;
      event.normal       = hit.normal;
      event.direction_i  = -ray.direction;
      event.direction_o  = sample.direction_o;
      event.weight      *= bsdf / psa_pdf;
      events.push_back(event);

      ray = ray_type(hit.position, sample.direction_o);
      bsdf = sample.bsdf;
      psa_pdf = sample.psa_probability;

      auto const russian_roulette = (bsdf / psa_pdf).max();
      if (sampler->uniform<radiant_value_type>() >= russian_roulette) {
        break;
      }
      psa_pdf *= std::min<radiant_value_type>(1, russian_roulette);
    }

    return events;
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
};

}
}
}
