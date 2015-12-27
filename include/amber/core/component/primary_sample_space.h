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
  using seed_type   = typename BaseSampler::seed_type;
  using result_type = typename BaseSampler::result_type;

private:
  using timestamp_type = std::uint_fast64_t;

  struct Sample
  {
    timestamp_type timestamp;
    result_type value;

    Sample() noexcept;
    Sample(timestamp_type const timestamp, result_type const value) noexcept;
  };

  BaseSampler sampler_;

  std::vector<Sample> samples_;
  std::vector<Sample> proposals_;

  timestamp_type last_large_step_time_;
  timestamp_type time_;

  bool large_step_;
  std::size_t pos_;

public:
  explicit PrimarySampleSpace(seed_type const seed);

  result_type const operator()();
  result_type const operator[](std::size_t const pos);
  void SetLargeStep();
  void Accept() noexcept;
  void Reject() noexcept;
  void ResetPosition() noexcept;

private:
  Sample GenerateSample();
  Sample MutateSample(Sample const& sample);
  result_type const Mutate(result_type value);
};

template <typename BaseSampler>
class PrimarySampleSpacePair
{
public:
  using seed_type   = typename BaseSampler::seed_type;
  using result_type = typename BaseSampler::result_type;

  class Reference : public Sampler
  {
    friend class PrimarySampleSpacePair;

  private:
    PrimarySampleSpace<BaseSampler>* pss_;
    std::size_t pos_;

    Reference(
      PrimarySampleSpace<BaseSampler>* const pss,
      std::size_t const pos
    ) noexcept;

  public:
    result_type const operator()();
  };

private:
  PrimarySampleSpace<BaseSampler> pss_;

  Reference light_;
  Reference eye_;

public:
  explicit PrimarySampleSpacePair(seed_type const seed);
  PrimarySampleSpacePair(PrimarySampleSpacePair<BaseSampler> const& pssp);

  PrimarySampleSpacePair<BaseSampler>& operator=(
    PrimarySampleSpacePair<BaseSampler> const& pssp
  );

  Reference& light() noexcept { return light_; }
  Reference& eye() noexcept { return eye_; }

  void SetLargeStep();
  void Accept() noexcept;
  void Reject() noexcept;
  void ResetPosition() noexcept;

private:
  void ResetReference() noexcept;
};

using MTPrimarySampleSpace     = PrimarySampleSpace<MTSampler>;
using MTPrimarySampleSpacePair = PrimarySampleSpacePair<MTSampler>;

template <typename BaseSampler>
PrimarySampleSpace<BaseSampler>::PrimarySampleSpace(
  seed_type const seed
)
: sampler_(seed)
, samples_()
, proposals_()
, last_large_step_time_(0)
, time_(0)
, large_step_(false)
, pos_(0)
{}

template <typename BaseSampler>
auto
PrimarySampleSpace<BaseSampler>::operator()()
-> result_type const
{
  auto const position = pos_++;

  if (position < proposals_.size()) {
    return proposals_.at(position).value;
  }

  if (large_step_ || position >= samples_.size()) {
    proposals_.emplace_back(GenerateSample());
    return proposals_.back().value;
  }

  auto& sample = samples_.at(position);

  if (sample.timestamp < last_large_step_time_) {
    proposals_.emplace_back(GenerateSample());
    return proposals_.back().value;
  }

  while (sample.timestamp + 1 < time_) {
    sample = MutateSample(sample);
  }

  proposals_.emplace_back(MutateSample(sample));
  return proposals_.back().value;
}

template <typename BaseSampler>
auto
PrimarySampleSpace<BaseSampler>::operator[](std::size_t const pos)
-> result_type const
{
  while (pos >= proposals_.size()) {
    operator()();
  }

  return proposals_.at(pos).value;
}

template <typename BaseSampler>
void
PrimarySampleSpace<BaseSampler>::SetLargeStep()
{
  if (proposals_.size() > 0) {
    throw std::logic_error("PrimarySampleSpace::SetLargeStep: cannot be called after sampling");
  }

  large_step_ = true;
}

template <typename BaseSampler>
void
PrimarySampleSpace<BaseSampler>::Accept() noexcept
{
  samples_.resize(std::max(samples_.size(), proposals_.size()));
  std::copy(proposals_.begin(), proposals_.end(), samples_.begin());
  proposals_.clear();

  if (large_step_) {
    last_large_step_time_ = time_;
  }
  time_++;

  large_step_ = false;
  pos_ = 0;
}

template <typename BaseSampler>
void
PrimarySampleSpace<BaseSampler>::Reject() noexcept
{
  proposals_.clear();

  large_step_ = false;
  pos_ = 0;
}

template <typename BaseSampler>
void
PrimarySampleSpace<BaseSampler>::ResetPosition() noexcept
{
  pos_ = 0;
}

template <typename BaseSampler>
auto
PrimarySampleSpace<BaseSampler>::GenerateSample()
-> Sample
{
  return Sample(time_, sampler_());
}

template <typename BaseSampler>
auto
PrimarySampleSpace<BaseSampler>::MutateSample(Sample const& sample)
-> Sample
{
  return Sample(sample.timestamp + 1, Mutate(sample.value));
}

template <typename BaseSampler>
auto
PrimarySampleSpace<BaseSampler>::Mutate(result_type value)
-> result_type const
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

template <typename BaseSampler>
PrimarySampleSpace<BaseSampler>::Sample::Sample() noexcept
: timestamp(0)
, value(0)
{}

template <typename BaseSampler>
PrimarySampleSpace<BaseSampler>::Sample::Sample(
  timestamp_type const timestamp,
  result_type const value
) noexcept
: timestamp(timestamp)
, value(value)
{}

template <typename BaseSampler>
PrimarySampleSpacePair<BaseSampler>::PrimarySampleSpacePair(
  seed_type const seed
)
: pss_(seed)
, light_(&pss_, 0)
, eye_(&pss_, 1)
{}

template <typename BaseSampler>
PrimarySampleSpacePair<BaseSampler>::PrimarySampleSpacePair(
  PrimarySampleSpacePair<BaseSampler> const& pssp
)
: pss_(pssp.pss_)
, light_(&pss_, 0)
, eye_(&pss_, 1)
{}

template <typename BaseSampler>
auto
PrimarySampleSpacePair<BaseSampler>::operator=(
  PrimarySampleSpacePair<BaseSampler> const& pssp
)
-> PrimarySampleSpacePair<BaseSampler>&
{
  pss_ = pssp.pss_;
  ResetReference();
  return *this;
}

template <typename BaseSampler>
void
PrimarySampleSpacePair<BaseSampler>::SetLargeStep()
{
  pss_.SetLargeStep();
  ResetReference();
}

template <typename BaseSampler>
void
PrimarySampleSpacePair<BaseSampler>::Accept() noexcept
{
  pss_.Accept();
  ResetReference();
}

template <typename BaseSampler>
void
PrimarySampleSpacePair<BaseSampler>::Reject() noexcept
{
  pss_.Reject();
  ResetReference();
}

template <typename BaseSampler>
void
PrimarySampleSpacePair<BaseSampler>::ResetPosition() noexcept
{
  pss_.ResetPosition();
  ResetReference();
}

template <typename BaseSampler>
void
PrimarySampleSpacePair<BaseSampler>::ResetReference() noexcept
{
  light_ = Reference(&pss_, 0);
  eye_ = Reference(&pss_, 1);
}

template <typename BaseSampler>
PrimarySampleSpacePair<BaseSampler>::Reference::Reference(
  PrimarySampleSpace<BaseSampler>* const pss,
  std::size_t const pos
) noexcept
: pss_(pss)
, pos_(pos)
{}

template <typename BaseSampler>
auto
PrimarySampleSpacePair<BaseSampler>::Reference::operator()()
-> result_type const
{
  auto const value = (*pss_)[pos_];
  pos_ += 2;
  return value;
}

}
}
}
