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
#include <mutex>
#include <thread>
#include <vector>

#include "core/component/bidirectional_path_tracing.h"
#include "core/component/multiple_importance_sampling.h"
#include "core/component/primary_sample_space.h"
#include "core/shader.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class PrimarySampleSpaceMLT : public Shader<Object>
{
private:
  using typename Shader<Object>::camera_type;
  using typename Shader<Object>::image_type;
  using typename Shader<Object>::scene_type;

  using hit_type           = typename Object::hit_type;
  using radiant_type       = typename Object::radiant_type;
  using radiant_value_type = typename Object::radiant_value_type;
  using ray_type           = typename Object::ray_type;
  using real_type          = typename Object::real_type;
  using vector3_type       = typename Object::vector3_type;

  using bdpt_type = component::BidirectionalPathTracing<Object>;

  struct Seed
  {
    component::PrimarySampleSpace<> pss_light, pss_eye;
    size_t x, y;
    radiant_type power;
    radiant_value_type contribution;

    Seed(camera_type const& camera,
         bdpt_type& bdpt,
         DefaultSampler<>& sampler)
      : pss_light(sampler()),
        pss_eye(sampler()),
        x(std::floor(pss_eye.uniform<real_type>(camera.imageWidth()))),
        y(std::floor(pss_eye.uniform<real_type>(camera.imageHeight()))),
        power(bdpt.connect(
          bdpt.lightTracing(&pss_light),
          bdpt.rayTracing(&pss_eye, camera, x, y),
          component::PowerHeuristic<radiant_value_type>())),
        contribution(Sum(power)) {
      pss_light.accept();
      pss_eye.accept();
    }

  };

  struct State
  {
    size_t x, y;
    radiant_type power;
    radiant_value_type contribution;
    radiant_value_type weight;

    explicit State(Seed const& seed) noexcept
      : x(seed.x),
        y(seed.y),
        power(seed.power),
        contribution(seed.contribution),
        weight(0) {}
  };

  size_t n_threads_, n_seeds_, n_mutations_;
  real_type p_large_step_;
  Progress progress_;

public:
  PrimarySampleSpaceMLT(
    size_t n_threads,
    size_t n_seeds,
    size_t n_mutations,
    real_type p_large_step
  ) noexcept
  : n_threads_(n_threads),
    n_seeds_(n_seeds),
    n_mutations_(n_mutations),
    p_large_step_(p_large_step),
    progress_(2)
  {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "PrimarySampleSpaceMLT(n_threads=" << n_threads_
      << ", n_seeds=" << n_seeds_
      << ", n_mutations=" << n_mutations_
      << ", p_large_step=" << p_large_step_
      << ")";
  }

  Progress const& progress() const noexcept { return progress_; }

  image_type operator()(scene_type const& scene, camera_type const& camera)
  {
    std::vector<std::thread> threads;
    std::mutex mtx;

    progress_.phase = "Generate seeds";
    progress_.current_phase = 1;
    progress_.current_job = 0;
    progress_.total_job = n_seeds_;

    std::vector<Seed> seeds;
    for (size_t i = 0; i < n_threads_; i++) {
      threads.emplace_back([&](){
        bdpt_type bdpt(scene);
        DefaultSampler<> sampler((std::random_device()()));
        while (progress_.current_job++ < progress_.total_job) {
          Seed seed(camera, bdpt, sampler);
          std::lock_guard<std::mutex> lock(mtx);
          seeds.push_back(seed);
        }
      });
    }
    while (!threads.empty()) {
      threads.back().join();
      threads.pop_back();
    }

    radiant_value_type b = 0;
    for (auto const& seed : seeds) {
      b += seed.contribution / n_seeds_;
    }

    progress_.phase = "Mutation";
    progress_.current_phase = 2;
    progress_.current_job = 0;
    progress_.total_job = n_mutations_;

    image_type image(camera.imageWidth(), camera.imageHeight());
    for (size_t i = 0; i < n_threads_; i++) {
      threads.emplace_back([&](){
        bdpt_type bdpt(scene);
        DefaultSampler<> sampler((std::random_device()()));
        image_type buffer(camera.imageWidth(), camera.imageHeight());
        auto seed = sampleSeed(seeds.begin(), seeds.end(), sampler);
        auto& pss_light = seed.pss_light;
        auto& pss_eye = seed.pss_eye;
        State state(seed);
        while (progress_.current_job++ < progress_.total_job) {
          auto const large_step = sampler.canonical() < p_large_step_;
          if (large_step) {
            pss_light.setLargeStep();
            pss_eye.setLargeStep();
          }
          auto proposal = propose(state, bdpt, camera, pss_light, pss_eye);
          auto const p_acceptance = std::min<radiant_value_type>(
            1,
            proposal.contribution / state.contribution);
          proposal.weight =
            (p_acceptance + large_step) /
              (proposal.contribution / b + p_large_step_) / n_mutations_;
          state.weight +=
            (1 - p_acceptance) /
              (state.contribution / b + p_large_step_) / n_mutations_;
          if (sampler.uniform<radiant_value_type>() < p_acceptance) {
            buffer.at(state.x, state.y) += state.power * state.weight;
            state = proposal;
            pss_light.accept();
            pss_eye.accept();
          } else {
            buffer.at(proposal.x, proposal.y) +=
              proposal.power * proposal.weight;
            pss_light.reject();
            pss_eye.reject();
          }
        }
        buffer.at(state.x, state.y) += state.power * state.weight;
        std::lock_guard<std::mutex> lock(mtx);
        for (size_t y = 0; y < camera.imageHeight(); y++) {
          for (size_t x = 0; x < camera.imageWidth(); x++) {
            image.at(x, y) += buffer.at(x, y) * camera.imageSize();
          }
        }
      });
    }
    while (!threads.empty()) {
      threads.back().join();
      threads.pop_back();
    }
    return image;
  }

private:
  template <typename InputIterator>
  Seed sampleSeed(
    InputIterator first,
    InputIterator last,
    DefaultSampler<>& sampler
  ) const
  {
    radiant_value_type c = 0;
    std::vector<radiant_value_type> cs;
    std::for_each(first, last, [&](auto const& seed){
      c += seed.contribution;
      cs.push_back(c);
    });
    auto const it = std::lower_bound(cs.begin(), cs.end(), sampler.uniform(c));
    return *std::next(first, std::distance(cs.begin(), it));
  }

  State propose(
    State state,
    bdpt_type& bdpt,
    camera_type const& camera,
    component::PrimarySampleSpace<>& pss_light,
    component::PrimarySampleSpace<>& pss_eye
  ) const
  {
    state.x = std::floor(pss_eye.uniform<real_type>(camera.imageWidth()));
    state.y = std::floor(pss_eye.uniform<real_type>(camera.imageHeight()));
    state.power = bdpt.connect(
      bdpt.lightTracing(&pss_light),
      bdpt.rayTracing(&pss_eye, camera, state.x, state.y),
      component::PowerHeuristic<radiant_value_type>()
    );
    state.contribution = Sum(state.power);
    return state;
  }
};

}
}
}
