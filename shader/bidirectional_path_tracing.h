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
#include <sstream>
#include <vector>

#include "shader/framework/bidirectional_path_tracing.h"
#include "shader/framework/light_sampler.h"
#include "shader/shader.h"

namespace amber {
namespace shader {

template <typename Acceleration>
class BidirectionalPathTracing : public Shader<Acceleration> {
private:
  using shader_type              = Shader<Acceleration>;

  using acceleration_type        = typename shader_type::acceleration_type;
  using camera_type              = typename shader_type::camera_type;
  using object_buffer_type       = typename shader_type::object_buffer_type;
  using object_type              = typename shader_type::object_type;
  using progress_const_reference = typename shader_type::progress_const_reference;
  using progress_reference       = typename shader_type::progress_reference;
  using progress_type            = typename shader_type::progress_type;
  using radiant_type             = typename shader_type::radiant_type;
  using real_type                = typename shader_type::real_type;

  using bdpt_type                = framework::BidirectionalPathTracing<acceleration_type>;
  using light_sampler_type       = framework::LightSampler<object_type>;

  size_t n_thread_, spp_;

public:
  BidirectionalPathTracing(size_t n_thread, size_t spp) noexcept
    : n_thread_(n_thread), spp_(spp) {}

  std::string to_string() const {
    std::stringstream ss;
    ss
      << "RealType: " << sizeof(real_type) * 8 << "bit" << std::endl
      << "Shader: BidirectionalPathTracing(n_thread=" << n_thread_
      << ", spp=" << spp_
      << ")";
    return ss.str();
  }

  progress_const_reference render(const object_buffer_type& objects,
                                  const camera_type& camera) const {
    // TODO refactor
    std::vector<std::thread> threads;
    std::promise<progress_reference> promise;
    auto future = promise.get_future().share();
    auto current = std::make_shared<std::atomic<size_t>>(0);
    auto pixels = std::make_shared<std::vector<size_t>>(camera.image_pixels());
    std::iota(pixels->begin(), pixels->end(), 0);
    std::shuffle(pixels->begin(), pixels->end(), std::random_device());

    const auto acceleration = std::make_shared<acceleration_type>(objects);
    const auto light_sampler = std::make_shared<light_sampler_type>(objects.begin(), objects.end());
    const auto bdpt = std::make_shared<bdpt_type>(acceleration, light_sampler);

    for (size_t i = 0; i < n_thread_; i++) {
      using namespace std::placeholders;
      threads.push_back(std::thread(std::bind(&BidirectionalPathTracing::process, this, _1, _2, _3, _4, _5), bdpt, camera, future, current, pixels));
    }

    promise.set_value(std::make_shared<progress_type>(
      spp_ * camera.image_pixels(),
      std::move(threads)
    ));

    return future.get();
  }

private:
  void process(std::shared_ptr<bdpt_type> bdpt,
               const camera_type& camera,
               std::shared_future<progress_reference> future,
               std::shared_ptr<std::atomic<size_t>> current,
               std::shared_ptr<std::vector<size_t>> pixels) const {
    // TODO refactor
    auto progress = future.get();
    DefaultSampler<> sampler;

    for (size_t i = (*current)++; i < camera.image_pixels(); i = (*current)++) {
      auto x = pixels->at(i) % camera.image_width();
      auto y = pixels->at(i) / camera.image_height();
      radiant_type power;
      for (size_t j = 0; j < spp_; j++) {
        power += bdpt->connect(bdpt->lightPathTracing(&sampler),
                               bdpt->eyePathTracing(&sampler, camera, x, y),
                               framework::PowerHeuristic<radiant_type>());
        progress->done(1);
      }
      camera.expose(x, y, power / spp_);
    }

    progress->end();
  }
};

}
}
