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

#include "sampler.h"

namespace amber {
namespace component {

template <typename BaseSampler = DefaultSampler<>>
class PrimarySampleSpace : public Sampler {
public:
  using result_type    = typename BaseSampler::result_type;

private:
  using real_type      = typename Sampler::real_type;
  using size_type      = std::size_t;
  using timestamp_type = std::uint_fast64_t;

  struct Sample {
    timestamp_type timestamp;
    real_type value;

    Sample(timestamp_type timestamp, real_type value) noexcept
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
  PrimarySampleSpace() noexcept
    : sampler_(), samples_(), logs_(),
      large_step_(false), large_step_time_(0), time_(0), pos_(0) {}

  explicit PrimarySampleSpace(result_type seed) noexcept
    : sampler_(seed), samples_(), logs_(),
      large_step_(false), large_step_time_(0), time_(0), pos_(0) {}

  void setLargeStep() noexcept { large_step_ = true; }

  void accept() noexcept {
    logs_.clear();
    large_step_ = false;
    large_step_time_ = time_;
    pos_ = 0;
    time_++;
  }

  void reject() noexcept {
    std::copy(logs_.begin(), logs_.end(), samples_.begin());
    while (!samples_.empty() && samples_.back().timestamp == time_) {
      samples_.pop_back();
    }
    logs_.clear();
    large_step_ = false;
    pos_ = 0;
  }

  void clear() noexcept {
    samples_.clear();
    logs_.clear();
    large_step_ = false;
    large_step_time_ = 0;
    pos_ = 0;
    time_ = 0;
  }

  real_type canonical() {
    const auto position = pos_++;

    if (position >= samples_.size()) {
      while (position >= samples_.size()) {
        samples_.emplace_back(time_, sampler_.canonical());
      }
      return samples_.back().value;
    }

    auto& sample = samples_.at(position);
    if (large_step_ || sample.timestamp < large_step_time_) {
      logs_.push_back(sample);
      sample.timestamp = time_;
      sample.value = sampler_.canonical();
    } else {
      while (sample.timestamp < time_) {
        if (sample.timestamp == time_ - 1) {
          logs_.push_back(sample);
        }
        sample.timestamp++;
        sample.value = mutate(sample.value);
      }
    }

    return sample.value;
  }

private:
  real_type mutate(real_type value) {
    const real_type s1 = 1. / 1024;
    const real_type s2 = 1. / 64;
    const auto dv = s2 * std::exp(-std::log(s2 / s1) * sampler_.canonical());
    if (sampler_.canonical() < 0.5) {
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

}
}
