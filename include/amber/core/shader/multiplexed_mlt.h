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
#include <numeric>
#include <thread>
#include <unordered_map>

#include "core/component/bidirectional_path_tracing.h"
#include "core/component/image_average_buffer.h"
#include "core/component/multiple_importance_sampling.h"
#include "core/component/primary_sample_space.h"
#include "core/shader.h"

namespace amber {
namespace core {
namespace shader {

template <typename Object>
class MultiplexedMLT : public Shader<Object>
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

  using event_type       = component::SubpathEvent<radiant_type, real_type>;
  using path_buffer_type = component::PathBuffer<radiant_type, real_type>;

  struct State
  {
    std::size_t x, y;
    radiant_type measurement;
    radiant_value_type weight;

    State() noexcept
    : x(0)
    , y(0)
    , measurement(std::numeric_limits<radiant_value_type>::min())
    , weight(0)
    {}

    State(
      std::size_t const x,
      std::size_t const y,
      radiant_type const measurement
    ) noexcept
    : x(x)
    , y(y)
    , measurement(measurement)
    , weight(0)
    {}
  };

  struct Chain
  : public component::MTPrimarySampleSpacePair
  , public State
  {
    using component::MTPrimarySampleSpacePair::seed_type;

    explicit Chain(seed_type const seed)
    : component::MTPrimarySampleSpacePair(seed)
    , State()
    {}
  };

  using ChainMap = std::unordered_map<std::size_t, Chain>;

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

  using NormalizationFactorMap =
    std::unordered_map<std::size_t, NormalizationFactor>;

  static NormalizationFactorMap
  Merge(NormalizationFactorMap const& xs, NormalizationFactorMap const& ys)
  {
    NormalizationFactorMap bs;

    for (auto const& p : xs) {
      bs[p.first] += p.second;
    }

    for (auto const& p : ys) {
      bs[p.first] += p.second;
    }

    return bs;
  }


  std::size_t n_seeds_;
  real_type p_large_step_;

public:
  MultiplexedMLT(
    std::size_t const n_seeds,
    real_type const p_large_step
  ) noexcept
  : n_seeds_(n_seeds)
  , p_large_step_(p_large_step)
  {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "MultiplexedMLT(n_seeds=" << n_seeds_
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
    auto bs =
      std::make_shared<NormalizationFactorMap>(EstimateNormalizationFactor(
        scene,
        camera
      ));

    component::ImageAverageBuffer<radiant_type>
      image(camera.ImageWidth(), camera.ImageHeight());
    {
      std::mutex mtx;
      std::unordered_map<std::thread::id, std::shared_ptr<ChainMap>> chain_maps;

      IterateParallel(ctx, [&](auto const&){
        MTSampler sampler((std::random_device()()));
        path_buffer_type path_buffer;
        NormalizationFactorMap bs_buffer;

        auto const bs_snapshot = bs;

        std::shared_ptr<ChainMap> chains;
        {
          auto const id = std::this_thread::get_id();
          {
            std::lock_guard<std::mutex> lock(mtx);
            if (chain_maps.count(id)) {
              chains = chain_maps.at(id);
            }
          }
          if (!chains) {
            chains = std::make_shared<ChainMap>(GenerateChains(
              scene,
              camera,
              *bs_snapshot,
              path_buffer,
              bs_buffer,
              sampler
            ));

            std::lock_guard<std::mutex> lock(mtx);
            chain_maps.emplace(id, chains);
          }
        }

        image_type image_buffer(camera.ImageWidth(), camera.ImageHeight());
        for (std::size_t i = 0; i < camera.ImageSize(); i++) {
          Sample(
            scene,
            camera,
            *bs_snapshot,
            path_buffer,
            *chains,
            image_buffer,
            bs_buffer,
            sampler
          );
        }
        for (auto& p : *chains) {
          auto& chain = p.second;
          AccumulateContribution(chain, image_buffer);
        }

        std::lock_guard<std::mutex> lock(mtx);
        image.Buffer(std::move(image_buffer));
        bs = std::make_shared<NormalizationFactorMap>(Merge(*bs, bs_buffer));
      });
    }

    return image;
  }

private:
  NormalizationFactorMap
  EstimateNormalizationFactor(
    scene_type const& scene,
    camera_type const& camera
  ) const
  {
    NormalizationFactorMap bs;

    MTSampler sampler((std::random_device()()));
    path_buffer_type path_buffer;

    for (std::size_t v = 0; v < camera.ImageHeight(); v++) {
      for (std::size_t u = 0; u < camera.ImageWidth(); u++) {
        auto const light_path =
          component::GenerateLightSubpath(scene, camera, sampler);
        auto const eye_path =
          component::GenerateEyeSubpath(scene, camera, u, v, sampler);
        for (std::size_t s = 0; s <= light_path.size(); s++) {
          for (std::size_t t = 0; t <= eye_path.size(); t++) {
            auto contribution = component::Connect(
              scene,
              camera,
              s == 0 ? nullptr : &light_path.at(s - 1),
              t == 0 ? nullptr : &eye_path.at(t - 1)
            );

            if (!contribution) {
              continue;
            }

            path_buffer.Buffer(scene, camera, light_path, eye_path, s, t);
            contribution.measurement *=
              component::PowerHeuristic<radiant_value_type>()(
                path_buffer.p_technique.begin(),
                path_buffer.p_technique.end()
              );

            if (contribution.pixel) {
              contribution.measurement /= camera.ImageSize();
            }

            auto const k = s + t - 1;
            bs[k].contribution += Sum(contribution.measurement);
          }
        }
      }
    }

    for (auto& p : bs) {
      auto const& k = p.first;
      auto& b = p.second;
      b.n_samples = camera.ImageSize() * (k + 2);
    }

    return bs;
  }

  ChainMap
  GenerateChains(
    scene_type const& scene,
    camera_type const& camera,
    NormalizationFactorMap const& bs,
    path_buffer_type& path_buffer,
    NormalizationFactorMap& bs_buffer,
    MTSampler& sampler
  ) const
  {
    ChainMap chains;
    for (auto const& p : bs) {
      auto const& k = p.first;
      chains.emplace(k, Chain(sampler.engine()()));
    }

    image_type image_buffer(camera.ImageWidth(), camera.ImageHeight());
    for (std::size_t i = 0; i < n_seeds_; i++) {
      Sample(
        scene,
        camera,
        bs,
        path_buffer,
        chains,
        image_buffer,
        bs_buffer,
        sampler
      );
    }
    for (auto& p : chains) {
      auto& chain = p.second;
      chain.weight = 0;
    }

    return chains;
  }

  void
  Sample(
    scene_type const& scene,
    camera_type const& camera,
    NormalizationFactorMap const& bs,
    path_buffer_type& path_buffer,
    ChainMap& chains,
    image_type& buffer,
    NormalizationFactorMap& bs_buffer,
    Sampler& sampler
  ) const
  {
    std::size_t k;
    radiant_value_type p_k;
    std::tie(k, p_k) = SamplePathLength(bs, sampler);

    auto& chain = chains.at(k);

    auto const large_step = Uniform<real_type>(sampler) < p_large_step_;
    if (large_step) {
      chain.SetLargeStep();
    }

    auto proposal = ProposeState(scene, camera, k, path_buffer, chain);
    if (large_step) {
      auto& b_buffer = bs_buffer[k];
      b_buffer.contribution += Sum(proposal.measurement);
      b_buffer.n_samples++;
    }

    auto const p_acceptance = std::min<radiant_value_type>(
      1,
      Sum(proposal.measurement) / Sum(chain.measurement)
    );

    auto const b = bs.at(k);
    proposal.weight =
      (k + 2) / p_k *
      (p_acceptance + large_step) /
      (Sum(proposal.measurement) / b + p_large_step_);
    chain.weight +=
      (k + 2) / p_k *
      (1 - p_acceptance) /
      (Sum(chain.measurement) / b + p_large_step_);

    if (Uniform<radiant_value_type>(sampler) < p_acceptance) {
      // accepted
      AccumulateContribution(chain, buffer);
      chain.State::operator=(proposal);
      chain.Accept();
    } else {
      // rejected
      AccumulateContribution(proposal, buffer);
      chain.Reject();
    }
  }

  std::tuple<std::size_t, radiant_value_type>
  SamplePathLength(
    NormalizationFactorMap const& bs,
    Sampler& sampler
  ) const
  {
    auto const sum_b = std::accumulate(
      bs.begin(),
      bs.end(),
      static_cast<radiant_value_type>(0),
      [](auto const& acc, auto const& p){ return acc + p.second; }
    );
    auto key = Uniform(sum_b, sampler);

    for (auto const& p : bs) {
      auto const k = p.first;
      auto const b = p.second;
      key -= b;
      if (key < 0) {
        return std::make_tuple(k, b / sum_b);
      }
    }

    return SamplePathLength(bs, sampler);
  }

  State
  ProposeState(
    scene_type const& scene,
    camera_type const& camera,
    std::size_t const k,
    path_buffer_type& path_buffer,
    component::MTPrimarySampleSpacePair& pss
  ) const
  {
    std::size_t const s = Uniform<real_type>(k + 2, pss.light());
    std::size_t const t = k + 1 - s;

    auto const light_path =
      component::GenerateLightSubpath(scene, camera, s, pss.light());
    if (light_path.size() != s) {
      return State();
    }

    std::size_t u, v;
    std::vector<event_type> eye_path;
    std::tie(eye_path, u, v) =
      component::GenerateEyeSubpath(scene, camera, t, pss.eye());
    if (eye_path.size() != t) {
      return State();
    }

    auto const contribution = component::Connect(
      scene,
      camera,
      s == 0 ? nullptr : &light_path.at(s - 1),
      t == 0 ? nullptr : &eye_path.at(t - 1)
    );
    if (!contribution) {
      return State();
    }

    auto const weight = component::BidirectionalMISWeight(
      scene,
      camera,
      light_path,
      eye_path,
      s,
      t,
      path_buffer,
      component::PowerHeuristic<radiant_value_type>()
    );

    if (contribution.pixel) {
      return State(
        std::get<0>(*contribution.pixel),
        std::get<1>(*contribution.pixel),
        contribution.measurement * weight / camera.ImageSize()
      );
    } else {
      return State(u, v, contribution.measurement * weight);
    }
  }

  void
  AccumulateContribution(
    State& state,
    image_type& buffer
  ) const noexcept
  {
    buffer.at(state.x, state.y) += state.measurement * state.weight;
    state.weight = 0;
  }
};

}
}
}
