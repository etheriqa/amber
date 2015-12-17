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
#include <cmath>
#include <vector>

#include "core/sampler.h"

namespace amber {
namespace core {
namespace component {

template <typename BaseSampler>
class PrimarySampleSpace : public Sampler
{
public:
  using seed_type = typename BaseSampler::seed_type;

private:
  using typename Sampler::result_type;

  using size_type      = std::size_t;
  using timestamp_type = std::uint_fast64_t;

  struct Sample
  {
    timestamp_type timestamp;
    result_type value;

    Sample(timestamp_type timestamp, result_type value) noexcept
    : timestamp(timestamp), value(value) {}
  };

  BaseSampler sampler_;
  std::vector<Sample> samples_;
  std::vector<Sample> logs_;
  bool large_step_;
  timestamp_type large_step_time_;
  timestamp_type time_;
  size_type pos_;

public:
  explicit PrimarySampleSpace(seed_type seed) noexcept
  : sampler_(seed)
  , samples_()
  , logs_()
  , large_step_(false)
  , large_step_time_(0)
  , time_(0)
  , pos_(0)
  {}

  void SetLargeStep() noexcept { large_step_ = true; }
  void UnsetLargeStep() noexcept { large_step_ = false; }

  void Accept() noexcept
  {
    logs_.clear();
    large_step_ = false;
    large_step_time_ = time_;
    pos_ = 0;
    time_++;
  }

  void Reject() noexcept
  {
    std::copy(logs_.begin(), logs_.end(), samples_.begin());
    while (!samples_.empty() && samples_.back().timestamp == time_) {
      samples_.pop_back();
    }
    logs_.clear();
    large_step_ = false;
    pos_ = 0;
  }

  void Clear() noexcept
  {
    samples_.clear();
    logs_.clear();
    large_step_ = false;
    large_step_time_ = 0;
    pos_ = 0;
    time_ = 0;
  }

  result_type const operator()()
  {
    auto const position = pos_++;

    if (position >= samples_.size()) {
      while (position >= samples_.size()) {
        samples_.emplace_back(time_, sampler_());
      }
      return samples_.back().value;
    }

    auto& sample = samples_.at(position);
    if (large_step_ || sample.timestamp < large_step_time_) {
      logs_.push_back(sample);
      sample.timestamp = time_;
      sample.value = sampler_();
    } else {
      while (sample.timestamp < time_) {
        if (sample.timestamp == time_ - 1) {
          logs_.push_back(sample);
        }
        sample.timestamp++;
        sample.value = Mutate(sample.value);
      }
    }

    return sample.value;
  }

private:
  result_type const Mutate(result_type value)
  {
    auto const s1 = static_cast<result_type>(1) / 1024;
    auto const s2 = static_cast<result_type>(1) / 64;
    auto const dv = s2 * std::exp(-std::log(s2 / s1) * sampler_());
    if (sampler_() < 0.5) {
      value += dv;
      if (value >= 1) {
        value -= 1;
      }
    } else {
      value -= dv;
      if (value < 0) {
        value += 1;
      }
    }
    return value;
  }
};

template <typename BaseSampler>
class PrimarySampleSpacePair
{
public:
  using seed_type = typename BaseSampler::seed_type;

private:
  PrimarySampleSpace<BaseSampler> light_, eye_;

public:
  PrimarySampleSpacePair(
    seed_type const light_seed,
    seed_type const eye_seed
  )
  : light_(light_seed)
  , eye_(eye_seed)
  {}

  PrimarySampleSpace<BaseSampler>& light() noexcept { return light_; }
  PrimarySampleSpace<BaseSampler>& eye() noexcept { return eye_; }

  void SetLargeStep() noexcept
  {
    light_.SetLargeStep();
    eye_.SetLargeStep();
  }

  void UnsetLargeStep() noexcept
  {
    light_.UnsetLargeStep();
    eye_.UnsetLargeStep();
  }

  void Accept() noexcept
  {
    light_.Accept();
    eye_.Accept();
  }

  void Reject() noexcept
  {
    light_.Reject();
    eye_.Reject();
  }
};

using MTPrimarySampleSpacePair = PrimarySampleSpacePair<MTSampler>;

}
}
}
