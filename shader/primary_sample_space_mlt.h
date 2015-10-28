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

  void write(std::ostream& os) const noexcept {
    os
      << "PrimarySampleSpaceMLT(n_thread=" << n_thread_
      << ", n_seed=" << n_seed_
      << ", n_mutation=" << n_mutation_
      << ", p_large_step=" << p_large_step_
      << ")";
  }

  progress_const_reference render(const object_buffer_type& objects,
                                  const camera_type& camera) const {
    // TODO refactor
    std::vector<std::thread> threads;
    std::promise<progress_reference> promise;
    auto future = promise.get_future().share();
    auto n_done = std::make_shared<std::atomic<size_t>>(0);

    const auto acceleration = std::make_shared<acceleration_type>(objects);
    const auto light_sampler = std::make_shared<light_sampler_type>(objects.begin(), objects.end());
    const auto bdpt = std::make_shared<bdpt_type>(acceleration, light_sampler);

    for (size_t i = 0; i < n_thread_; i++) {
      using namespace std::placeholders;
      threads.push_back(std::thread(std::bind(&PrimarySampleSpaceMLT::process, this, _1, _2, _3, _4), bdpt, camera, future, n_done));
    }

    promise.set_value(std::make_shared<progress_type>(
      n_mutation_,
      std::move(threads)
    ));

    return future.get();
  }

private:
  void process(std::shared_ptr<bdpt_type> bdpt,
               const camera_type& camera,
               std::shared_future<progress_reference> future,
               std::shared_ptr<std::atomic<size_t>> n_done) const {
    // TODO refactor
    auto progress = future.get();

    DefaultSampler<> sampler((std::random_device()()));
    framework::PrimarySampleSpace<> pss_light, pss_eye;
    size_t current_x, current_y;
    radiant_type current;
    radiant_value_type b;
    std::tie(pss_light, pss_eye, current_x, current_y, current, b) =
      preprocess(bdpt, camera, sampler);
    radiant_value_type current_contribution = current.sum();
    radiant_value_type current_weight = 0;

    for (size_t i = (*n_done)++; i < n_mutation_; i = (*n_done)++) {
      const auto large_step = sampler.canonical() < p_large_step_;
      if (large_step) {
        pss_light.setLargeStep();
        pss_eye.setLargeStep();
      }

      const size_t proposal_x =
        std::floor(pss_eye.uniform<real_type>(camera.imageWidth()));
      const size_t proposal_y =
        std::floor(pss_eye.uniform<real_type>(camera.imageHeight()));

      const auto proposal =
        bdpt->connect(bdpt->lightPathTracing(&pss_light),
                      bdpt->eyePathTracing(&pss_eye,
                                           camera,
                                           proposal_x,
                                           proposal_y));
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
        camera.expose(current_x, current_y, current * current_weight);
        current_x = proposal_x;
        current_y = proposal_y;
        current = proposal;
        current_contribution = proposal_contribution;
        current_weight = proposal_weight;
        pss_light.accept();
        pss_eye.accept();
      } else {
        camera.expose(proposal_x, proposal_y, proposal * proposal_weight);
        pss_light.reject();
        pss_eye.reject();
      }

      progress->done(1);
    }

    camera.expose(current_x, current_y, current * current_weight);
    progress->end();
  }

  std::tuple<framework::PrimarySampleSpace<>,
             framework::PrimarySampleSpace<>,
             size_t,
             size_t,
             radiant_type,
             radiant_value_type>
  preprocess(std::shared_ptr<bdpt_type> const& bdpt,
             camera_type const& camera,
             DefaultSampler<>& sampler) const {
    std::vector<std::tuple<framework::PrimarySampleSpace<>,
                           framework::PrimarySampleSpace<>,
                           size_t,
                           size_t,
                           radiant_type>> samples;
    for (size_t i = 0; i < n_seed_; i++) {
      framework::PrimarySampleSpace<> pss_light(sampler()),
                                      pss_eye(sampler());
      const size_t x =
        std::floor(pss_eye.uniform<real_type>(camera.imageWidth()));
      const size_t y =
        std::floor(pss_eye.uniform<real_type>(camera.imageHeight()));
      const auto power =
        bdpt->connect(bdpt->lightPathTracing(&pss_light),
                      bdpt->eyePathTracing(&pss_eye, camera, x, y));
      pss_light.accept();
      pss_eye.accept();
      samples.emplace_back(pss_light, pss_eye, x, y, power);
    }

    radiant_value_type contribution = 0;
    std::vector<radiant_value_type> contributions;
    for (const auto& sample : samples) {
      const auto& power = std::get<4>(sample);
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
