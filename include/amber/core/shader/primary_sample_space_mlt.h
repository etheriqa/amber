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
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include <boost/operators.hpp>

#include "core/component/bidirectional_path_tracing.h"
#include "core/component/image_average_buffer.h"
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
  using unit_vector3_type  = typename Object::unit_vector3_type;

  using path_buffer_type       = component::PathBuffer<radiant_type, real_type>;
  using path_contribution_type = component::PathContribution<radiant_type>;

  struct State
  {
    std::size_t x, y;
    radiant_type measurement;
    std::vector<path_contribution_type> light_image;
    radiant_value_type contribution;
    radiant_value_type weight;

    State() noexcept
    : x(0)
    , y(0)
    , measurement()
    , light_image()
    , contribution(0)
    , weight(0)
    {}
  };

  struct Chain
  : public component::MTPrimarySampleSpacePair
  , public State
  {
    using component::MTPrimarySampleSpacePair::seed_type;

    Chain(seed_type const light_seed, seed_type const eye_seed)
    : component::MTPrimarySampleSpacePair(light_seed, eye_seed)
    , State()
    {}
  };

  struct NormalizationFactor
  : private boost::addable<NormalizationFactor>
  {
    radiant_value_type contribution;
    std::size_t n_samples;

    NormalizationFactor() noexcept
    : contribution(0)
    , n_samples(0)
    {}

    operator radiant_value_type() const noexcept
    {
      return contribution / n_samples;
    }

    NormalizationFactor& operator+=(NormalizationFactor const& b) noexcept
    {
      contribution += b.contribution;
      n_samples += b.n_samples;
      return *this;
    }
  };

  std::size_t n_seeds_;
  real_type p_large_step_;

public:
  PrimarySampleSpaceMLT(
    std::size_t const n_seeds,
    real_type const p_large_step
  ) noexcept
  : n_seeds_(n_seeds)
  , p_large_step_(p_large_step)
  {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "PrimarySampleSpaceMLT(n_seeds=" << n_seeds_
      << ", p_large_step=" << p_large_step_
      << ")";
  }

  image_type
  operator()(
    scene_type const& scene,
    camera_type const& camera,
    Context& ctx
  )
  {
    auto b = std::make_shared<NormalizationFactor>();

    component::ImageAverageBuffer<radiant_type>
      image(camera.ImageWidth(), camera.ImageHeight());
    {
      std::mutex mtx;
      std::unordered_map<std::thread::id, std::shared_ptr<Chain>> chains;

      IterateParallel(ctx, [&](auto const&){
        MTSampler sampler((std::random_device()()));
        path_buffer_type path_buffer;

        std::shared_ptr<Chain> chain;
        {
          auto const id = std::this_thread::get_id();
          {
            std::lock_guard<std::mutex> lock(mtx);
            if (chains.count(id)) {
              chain = chains.at(id);
            }
          }
          if (!chain) {
            NormalizationFactor b_buffer;
            chain = std::make_shared<Chain>(GenerateChain(
              scene,
              camera,
              path_buffer,
              b_buffer,
              sampler
            ));

            std::lock_guard<std::mutex> lock(mtx);
            chains.emplace(id, chain);
            b = std::make_shared<NormalizationFactor>(*b + b_buffer);
          }
        }

        image_type image_buffer(camera.ImageWidth(), camera.ImageHeight());
        NormalizationFactor b_buffer;
        for (std::size_t i = 0; i < camera.ImageSize(); i++) {
          Sample(
            scene,
            camera,
            *b,
            path_buffer,
            *chain,
            image_buffer,
            b_buffer,
            sampler
          );
        }
        AccumulateContribution(*chain, image_buffer);

        std::lock_guard<std::mutex> lock(mtx);
        image.Buffer(std::move(image_buffer));
        b = std::make_shared<NormalizationFactor>(*b + b_buffer);
      });
    }

    return image;
  }

private:
  Chain
  GenerateChain(
    scene_type const& scene,
    camera_type const& camera,
    path_buffer_type& path_buffer,
    NormalizationFactor& b_buffer,
    MTSampler& sampler
  ) const
  {
    Chain chain(sampler.engine()(), sampler.engine()());

    image_type image_buffer(camera.ImageWidth(), camera.ImageHeight());
    for (std::size_t i = 0; i < n_seeds_; i++) {
      Sample(scene, camera, 1, path_buffer, chain, image_buffer, b_buffer, sampler);
    }
    chain.weight = 0;

    return chain;
  }

  void
  Sample(
    scene_type const& scene,
    camera_type const& camera,
    radiant_value_type const b,
    path_buffer_type& path_buffer,
    Chain& chain,
    image_type& image_buffer,
    NormalizationFactor& b_buffer,
    Sampler& sampler
  ) const
  {
    auto const large_step = Uniform<real_type>(sampler) < p_large_step_;
    if (large_step) {
      chain.SetLargeStep();
    }

    auto proposal = ProposeState(scene, camera, path_buffer, chain);
    if (large_step) {
      b_buffer.contribution += proposal.contribution;
      b_buffer.n_samples++;
    }

    auto const p_acceptance = std::min<radiant_value_type>(
      1,
      proposal.contribution / chain.contribution
    );

    proposal.weight =
      (p_acceptance + large_step) /
      (proposal.contribution / b + p_large_step_);
    chain.weight +=
      (1 - p_acceptance) /
      (chain.contribution / b + p_large_step_);

    if (Uniform<radiant_value_type>(sampler) < p_acceptance) {
      // accepted
      AccumulateContribution(chain, image_buffer);
      chain.State::operator=(proposal);
      chain.Accept();
    } else {
      // rejected
      AccumulateContribution(proposal, image_buffer);
      chain.Reject();
    }
  }

  State
  ProposeState(
    scene_type const& scene,
    camera_type const& camera,
    path_buffer_type& path_buffer,
    component::MTPrimarySampleSpacePair& pss
  ) const
  {
    State state;
    state.x = Uniform<real_type>(camera.ImageWidth(), pss.eye());
    state.y = Uniform<real_type>(camera.ImageHeight(), pss.eye());
    std::tie(state.measurement, state.light_image) = component::Combine(
      scene,
      camera,
      component::GenerateLightSubpath(scene, camera, pss.light()),
      component::GenerateEyeSubpath(scene, camera, state.x, state.y, pss.eye()),
      path_buffer,
      component::PowerHeuristic<radiant_value_type>()
    );
    state.contribution += Sum(state.measurement);
    for (auto const& contribution : state.light_image) {
      state.contribution += Sum(contribution.measurement);
    }
    return state;
  }

  void
  AccumulateContribution(
    State& state,
    image_type& image_buffer
  ) const noexcept
  {
    image_buffer.at(state.x, state.y) += state.measurement * state.weight;
    for (auto const& contribution : state.light_image) {
      auto const& x = std::get<0>(*contribution.pixel);
      auto const& y = std::get<1>(*contribution.pixel);
      image_buffer.at(x, y) += contribution.measurement * state.weight;
    }
    state.weight = 0;
  }
};

}
}
}
