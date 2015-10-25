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
#include "constant.h"
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
    radiant_value_type probability;
    radiant_type weight;
  };

  acceleration_ptr acceleration_;
  light_sampler_ptr light_sampler_;

public:
  BidirectionalPathTracing(const acceleration_ptr& acceleration,
                           const light_sampler_ptr& light_sampler) noexcept
   : acceleration_(acceleration), light_sampler_(light_sampler) {}

  template <typename Random>
  std::vector<Event> lightPathTracing(Random& random) const {
    const auto light = (*light_sampler_)(random);
    const auto sample = light.sample_initial_ray(random);
    const auto area_probability =
      light.emittance().sum() * kPI / light_sampler_->total_power().sum();
    const auto psa_probability = static_cast<radiant_value_type>(1 / kPI);

    Event event;
    event.object      = light;
    event.position    = sample.ray.origin;
    event.normal      = sample.normal;
    event.direction_i = vector3_type();
    event.direction_o = sample.ray.direction;
    event.probability = area_probability;
    event.weight      = light.emittance() / area_probability;

    return pathTracing(event, psa_probability, random);
  }

  template <typename Random, typename Camera>
  std::vector<Event> eyePathTracing(Random& random,
                                    const Camera& camera,
                                    size_t x,
                                    size_t y) const {
    // TODO importance, area_probability, psa_probability
    const auto importance = radiant_type(1);
    const auto sample = camera.sample_initial_ray(x, y, random);
    const auto area_probability = static_cast<radiant_value_type>(1);
    const auto psa_probability = static_cast<radiant_value_type>(1 / kPI);

    Event event;
    event.object      = Object();
    event.position    = sample.ray.origin;
    event.normal      = sample.normal;
    event.direction_i = vector3_type();
    event.direction_o = sample.ray.direction;
    event.probability = area_probability;
    event.weight      = importance / area_probability;

    return pathTracing(event, psa_probability, random);
  }

  template <typename MIS>
  radiant_type connect(const std::vector<Event>& light,
                       const std::vector<Event>& eye,
                       MIS mis) const noexcept {
    std::vector<std::vector<Sample<radiant_type>>>
      sample_map(light.size() + eye.size() - 1);

    for (size_t s = 0; s <= light.size(); s++) {
      for (size_t t = 0; t <= eye.size(); t++) {
        auto& samples = sample_map[s + t - 2];
        if (s + t < 2) {
          continue;
        } else if (s == 0 && t >= 2) {
          // zero light subpath vertices
          const auto& l = eye[t - 1];
          const auto& e = eye[t - 2];
          if (!l.object.isEmissive()) {
            continue;
          }
          if (dot(e.position - l.position, l.normal) <= 0) {
            continue;
          }
          samples.emplace_back(
            l.object.emittance() / kPI * l.weight,
            l.probability
          );
        } else if (s == 1 && t >= 2) {
          // one light subpath vertex
          const auto& l = light[s - 1];
          const auto& e = eye[t - 1];
          if (e.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          if (dot(e.position - l.position, l.normal) <= 0) {
            continue;
          }
          const auto direction_le = normalize(e.position - l.position);
          samples.emplace_back(
            l.weight /
              kPI *
              geometryFactor(l, e) *
              e.object.bsdf(-direction_le, e.direction_i, e.normal) *
              e.weight,
            l.probability * e.probability
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
        } else {
          const auto& l = light[s - 1];
          const auto& e = eye[t - 1];
          if (l.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          if (e.object.surfaceType() == material::SurfaceType::specular) {
            continue;
          }
          const auto direction_le = normalize(e.position - l.position);
          samples.emplace_back(
            l.weight *
              l.object.bsdf(l.direction_i, direction_le, l.normal) *
              geometryFactor(l, e) *
              e.object.bsdf(-direction_le, e.direction_i, e.normal) *
              e.weight,
            l.probability * e.probability
          );
        }
      }
    }

    return std::accumulate(sample_map.begin(), sample_map.end(), radiant_type(),
      [&mis](const auto& power, const auto& samples){
        return power + mis(samples.begin(), samples.end());
      });
  }

private:
  template <typename Random>
  std::vector<Event> pathTracing(Event event,
                                 radiant_value_type probability,
                                 Random& random) const {
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
        object.sampleScattering(event.weight * bsdf,
                                -ray.direction,
                                hit.normal,
                                random);
      const auto signed_geometry_factor =
        dot(ray.direction, event.normal) *
        dot(ray.direction, hit.normal) /
        (hit.distance * hit.distance);

      event.object       = object;
      event.position     = hit.position;
      event.normal       = hit.normal;
      event.direction_i  = -ray.direction;
      event.direction_o  = sample.direction_o;
      event.probability *= probability * std::abs(signed_geometry_factor);
      event.weight      *= bsdf / probability;
      events.push_back(event);

      ray = ray_type(hit.position, sample.direction_o);
      bsdf = sample.bsdf;
      probability = sample.psa_probability;

      const auto russian_roulette = (bsdf / probability).max();
      if (random.template uniform<radiant_value_type>() >= russian_roulette) {
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
