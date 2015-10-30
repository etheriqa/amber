/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <functional>
#include <future>
#include <vector>

#include "base/sampler.h"
#include "shader/shader.h"

namespace amber {
namespace shader {

template <typename Acceleration>
class PathTracing : public Shader<Acceleration> {
public:
  using shader_type              = Shader<Acceleration>;

  using acceleration_type        = typename shader_type::acceleration_type;
  using camera_type              = typename shader_type::camera_type;
  using hit_type                 = typename shader_type::hit_type;
  using object_buffer_type       = typename shader_type::object_buffer_type;
  using object_type              = typename shader_type::object_type;
  using progress_const_reference = typename shader_type::progress_const_reference;
  using progress_reference       = typename shader_type::progress_reference;
  using progress_type            = typename shader_type::progress_type;
  using radiant_type             = typename shader_type::radiant_type;
  using ray_type                 = typename shader_type::ray_type;
  using real_type                = typename shader_type::real_type;

private:
  const size_t m_n_thread, m_spp;

public:
  PathTracing(size_t n_thread, size_t spp) :
    m_n_thread(n_thread),
    m_spp(spp)
  {}

  void write(std::ostream& os) const noexcept {
    os
      << "PathTracing(n_thread=" << m_n_thread
      << ", spp=" << m_spp
      << ")";
  }

  progress_const_reference render(const object_buffer_type& objects, const camera_type& camera) const
  {
    // TODO refactor
    std::vector<std::thread> threads;
    std::promise<progress_reference> promise;
    auto future = promise.get_future().share();
    auto current = std::make_shared<std::atomic<size_t>>(0);
    auto pixels = std::make_shared<std::vector<size_t>>(camera.imageSize());
    std::iota(pixels->begin(), pixels->end(), 0);
    std::shuffle(pixels->begin(), pixels->end(), std::random_device());

    const auto acceleration = std::make_shared<acceleration_type>(objects.begin(), objects.end());

    for (size_t i = 0; i < m_n_thread; i++) {
      using namespace std::placeholders;
      threads.push_back(std::thread(std::bind(&PathTracing::process, this, _1, _2, _3, _4, _5), acceleration, camera, future, current, pixels));
    }

    promise.set_value(std::make_shared<progress_type>(
      m_spp * camera.imageSize(),
      std::move(threads)
    ));

    return future.get();
  }

private:
  void process(
    std::shared_ptr<acceleration_type> scene,
    const camera_type& camera,
    std::shared_future<progress_reference> future,
    std::shared_ptr<std::atomic<size_t>> current,
    std::shared_ptr<std::vector<size_t>> pixels
  ) const
  {
    // TODO refactor
    auto progress = future.get();
    DefaultSampler<> sampler;

    for (size_t i = (*current)++; i < camera.imageSize(); i = (*current)++) {
      auto x = pixels->at(i) % camera.imageWidth();
      auto y = pixels->at(i) / camera.imageHeight();
      radiant_type power;
      for (size_t j = 0; j < m_spp; j++) {
        power += sample_pixel(scene, camera, x, y, &sampler);
        progress->done(1);
      }
      camera.expose(x, y, power / m_spp);
    }

    progress->end();
  }

  radiant_type sample_pixel(
    const std::shared_ptr<acceleration_type>& scene,
    const camera_type& camera,
    size_t x,
    size_t y,
    Sampler *sampler
  ) const
  {
    radiant_type power, weight(1);
    ray_type ray = camera.sampleFirstRay(x, y, sampler);
    hit_type hit;
    object_type object;

    for (;;) {
      std::tie(hit, object) = scene->cast(ray);
      if (!hit) {
        break;
      }

      if (object.isEmissive() && dot(hit.normal, ray.direction) < 0) {
        power += weight * object.emittance();
      }

      const auto sample = object.sampleScatter(-ray.direction, hit.normal, sampler);
      ray = ray_type(hit.position, sample.direction_o);
      const auto reflectance = sample.bsdf / sample.psa_probability;
      weight *= reflectance;

      const auto p_russian_roulette = reflectance.max();
      if (sampler->uniform<real_type>() >= p_russian_roulette) {
        break;
      }
      weight /= std::min<real_type>(1, p_russian_roulette);
    }

    return power;
  }
};

}
}
