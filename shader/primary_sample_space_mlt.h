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
#include "shader/framework/primary_sample_space.h"
#include "shader/shader.h"

namespace amber {
namespace shader {

template <typename Acceleration>
class PrimarySampleSpaceMLT : public Shader<Acceleration> {
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
  using radiant_value_type       = typename shader_type::radiant_value_type;
  using real_type                = typename shader_type::real_type;

  using bdpt_type                = framework::BidirectionalPathTracing<acceleration_type>;
  using light_sampler_type       = framework::LightSampler<object_type>;


  size_t n_thread_, n_seed_, n_mutation_;
  real_type p_large_step_;

public:
  PrimarySampleSpaceMLT(size_t n_thread,
                        size_t n_seed,
                        size_t n_mutation,
                        real_type p_large_step) noexcept
    : n_thread_(n_thread),
      n_seed_(n_seed),
      n_mutation_(n_mutation),
      p_large_step_(p_large_step) {}

  std::string to_string() const {
    std::stringstream ss;
    ss
      << "RealType: " << sizeof(real_type) * 8 << "bit" << std::endl
      << "Shader: PrimarySampleSpaceMLT(n_thread=" << n_thread_
      << ", n_seed=" << n_seed_
      << ", n_mutation=" << n_mutation_
      << ", p_large_step=" << p_large_step_
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
      threads.push_back(std::thread(std::bind(&PrimarySampleSpaceMLT::process, this, _1, _2, _3, _4, _5), bdpt, camera, future, current, pixels));
    }

    promise.set_value(std::make_shared<progress_type>(
      n_mutation_ * camera.image_pixels(),
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
    DefaultSampler<> sampler((std::random_device()()));

    for (size_t i = (*current)++; i < camera.image_pixels(); i = (*current)++) {
      auto x = pixels->at(i) % camera.image_width();
      auto y = pixels->at(i) / camera.image_height();

      framework::PrimarySampleSpace<> pss_light, pss_eye;
      radiant_type current;
      radiant_value_type b;
      std::tie(pss_light, pss_eye, current, b) =
        preprocess(bdpt, camera, x, y, sampler);

      if (b == 0) {
        progress->done(n_mutation_);
        continue;
      }

      radiant_value_type current_contribution = current.sum();
      radiant_value_type current_weight = 0;

      radiant_type power;
      for (size_t j = 0; j < n_mutation_; j++) {
        const auto large_step = sampler.canonical() < p_large_step_;
        if (large_step) {
          pss_light.setLargeStep();
          pss_eye.setLargeStep();
        }

        const auto proposal =
          bdpt->connect(bdpt->lightPathTracing(&pss_light),
                        bdpt->eyePathTracing(&pss_eye, camera, x, y),
                        framework::PowerHeuristic<radiant_type>());
        const auto proposal_contribution = proposal.sum();
        const auto p_acceptance =
          std::min<radiant_value_type>(1,
                                       proposal_contribution /
                                         current_contribution);
        const auto proposal_weight =
          (p_acceptance + large_step) /
            (proposal_contribution / b + p_large_step_) / n_mutation_;
        current_weight +=
          (1 - p_acceptance) /
            (current_contribution / b + p_large_step_) / n_mutation_;

        if (sampler.uniform<radiant_value_type>() < p_acceptance) {
          power += current * current_weight;
          current = proposal;
          current_contribution = proposal_contribution;
          current_weight = proposal_weight;
          pss_light.accept();
          pss_eye.accept();
        } else {
          power += proposal * proposal_weight;
          pss_light.reject();
          pss_eye.reject();
        }

        progress->done(1);
      }
      power += current * current_weight;
      camera.expose(x, y, power);
    }

    progress->end();
  }

  std::tuple<framework::PrimarySampleSpace<>,
             framework::PrimarySampleSpace<>,
             radiant_type,
             radiant_value_type>
  preprocess(std::shared_ptr<bdpt_type> const& bdpt,
             camera_type const& camera,
             size_t x,
             size_t y,
             DefaultSampler<>& sampler) const {
    std::vector<std::tuple<framework::PrimarySampleSpace<>,
                           framework::PrimarySampleSpace<>,
                           radiant_type>> samples;
    for (size_t i = 0; i < n_seed_; i++) {
      framework::PrimarySampleSpace<> pss_light(sampler()),
                                      pss_eye(sampler());
      const auto power =
        bdpt->connect(bdpt->lightPathTracing(&pss_light),
                      bdpt->eyePathTracing(&pss_eye, camera, x, y),
                      framework::PowerHeuristic<radiant_type>());
      pss_light.accept();
      pss_eye.accept();
      samples.emplace_back(pss_light, pss_eye, power);
    }

    radiant_value_type contribution = 0;
    std::vector<radiant_value_type> contributions;
    for (const auto& sample : samples) {
      const auto& power = std::get<2>(sample);
      contribution += power.sum();
      contributions.push_back(contribution);
    }

    const auto pos =
      std::distance(contributions.begin(),
                    std::lower_bound(contributions.begin(),
                                     contributions.end(),
                                     sampler.uniform(contribution)));
    return std::tuple_cat(samples.at(pos),
                          std::make_tuple(contribution / n_seed_));
  }
};

}
}
