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
#include "shader/framework/multiple_importance_sampling.h"

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
    radiant_value_type log_probability;
    radiant_type weight;
  };

  acceleration_ptr acceleration_;
  light_sampler_ptr light_sampler_;

public:
  BidirectionalPathTracing(const acceleration_ptr& acceleration,
                           const light_sampler_ptr& light_sampler) noexcept
   : acceleration_(acceleration), light_sampler_(light_sampler) {}

  std::vector<Event> lightPathTracing(Sampler *sampler) const {
    const auto light = (*light_sampler_)(sampler);
    const auto ray = light.sampleFirstRay(sampler);
    const auto area_probability =
      light.emittance().sum() * kPI / light_sampler_->total_power().sum();
    const auto psa_probability = static_cast<radiant_value_type>(1 / kPI);

    Event event;
    event.object          = light;
    event.position        = ray.origin;
    event.normal          = ray.normal;
    event.direction_i     = vector3_type();
    event.direction_o     = ray.direction;
    event.log_probability = std::log(area_probability);
    event.weight          = light.emittance() * kPI / area_probability;

    return pathTracing(event, psa_probability, sampler);
  }

  template <typename Camera>
  std::vector<Event> eyePathTracing(Sampler *sampler,
                                    const Camera& camera,
                                    size_t x,
                                    size_t y) const {
    // TODO importance, area_probability, psa_probability
    const auto importance = radiant_type(1);
    const auto ray = camera.sampleFirstRay(x, y, sampler);
    const auto area_probability = static_cast<radiant_value_type>(1);
    const auto psa_probability = static_cast<radiant_value_type>(1 / kPI);

    Event event;
    event.object          = Object();
    event.position        = ray.origin;
    event.normal          = ray.normal;
    event.direction_i     = vector3_type();
    event.direction_o     = ray.direction;
    event.log_probability = std::log(area_probability);
    event.weight          = importance / area_probability;

    return pathTracing(event, psa_probability, sampler);
  }

  template <typename MIS>
  radiant_type connect(const std::vector<Event>& light,
                       const std::vector<Event>& eye,
                       const MIS& mis) const noexcept {
    std::vector<std::vector<Sample<radiant_type>>>
      sample_map(light.size() + eye.size() - 1);

    for (size_t s = 0; s <= light.size(); s++) {
      for (size_t t = 0; t <= eye.size(); t++) {
        if (s + t < 2) {
          continue;
        }
        auto& samples = sample_map.at(s + t - 2);
        if (s >= 2 && t >= 2) {
          const auto& l = light.at(s - 1);
          const auto& e = eye.at(t - 1);
          if (l.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          if (e.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          const auto geometry_factor = geometryFactor(l, e);
          if (geometry_factor == 0) {
            continue;
          }
          const auto direction_le = normalize(e.position - l.position);
          samples.emplace_back(
            l.weight *
              l.object.bsdf(l.direction_i, direction_le, l.normal) *
              geometry_factor *
              e.object.bsdf(-direction_le, e.direction_i, e.normal) *
              e.weight,
            l.log_probability + e.log_probability
          );
        } else if (s == 0 && t >= 2) {
          // zero light subpath vertices
          const auto& l = eye.at(t - 1);
          const auto& e = eye.at(t - 2);
          if (!l.object.isEmissive()) {
            continue;
          }
          if (dot(e.position - l.position, l.normal) <= 0) {
            continue;
          }
          samples.emplace_back(
            l.object.emittance() * l.weight,
            l.log_probability
          );
        } else if (s == 1 && t >= 2) {
          // one light subpath vertex
          const auto& l = light.at(s - 1);
          const auto& e = eye.at(t - 1);
          if (e.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          if (dot(e.position - l.position, l.normal) <= 0) {
            continue;
          }
          const auto geometry_factor = geometryFactor(l, e);
          if (geometry_factor == 0) {
            continue;
          }
          const auto direction_le = normalize(e.position - l.position);
          samples.emplace_back(
            l.weight *
              (1 / kPI) *
              geometry_factor *
              e.object.bsdf(-direction_le, e.direction_i, e.normal) *
              e.weight,
            l.log_probability + e.log_probability
          );
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
      }
    }

    return std::accumulate(sample_map.begin(), sample_map.end(), radiant_type(),
      [&mis](const auto& power, const auto& samples){
        return power + mis(samples.begin(), samples.end());
      });
  }

private:
  std::vector<Event> pathTracing(Event event,
                                 radiant_value_type probability,
                                 Sampler *sampler) const {
    std::vector<Event> events;
    events.reserve(16);
    events.push_back(event);

    ray_type ray(event.position, event.direction_o);
    radiant_type bsdf(probability);
    hit_type hit;
    Object object;

    for (;;) {
      std::tie(hit, object) = acceleration_->cast(ray);
      if (!hit) {
        break;
      }

      const auto sample =
        object.sampleScatter(event.weight * bsdf,
                                -ray.direction,
                                hit.normal,
                                sampler);
      const auto log_geometry_factor =
        std::log(std::abs(dot(ray.direction, event.normal) *
                          dot(ray.direction, hit.normal) /
                          (hit.distance * hit.distance)));

      event.object           = object;
      event.position         = hit.position;
      event.normal           = hit.normal;
      event.direction_i      = -ray.direction;
      event.direction_o      = sample.direction_o;
      event.log_probability += std::log(probability) + log_geometry_factor;
      event.weight          *= bsdf / probability;
      events.push_back(event);

      ray = ray_type(hit.position, sample.direction_o);
      bsdf = sample.bsdf;
      probability = sample.psa_probability;

      const auto russian_roulette = (bsdf / probability).max();
      if (sampler->uniform<radiant_value_type>() >= russian_roulette) {
        break;
      }
      probability *= std::min<radiant_value_type>(1, russian_roulette);
    }

    return events;
  }

  radiant_value_type geometryFactor(const Event& x,
                                    const Event& y) const noexcept {
    ray_type ray(x.position, y.position - x.position);
    if (!acceleration_->test_visibility(ray, y.object)) {
      return 0;
    }

    return std::abs(dot(ray.direction, x.normal) *
                    dot(ray.direction, y.normal) /
                    (y.position - x.position).squaredLength());
  }
};

}
}
}
