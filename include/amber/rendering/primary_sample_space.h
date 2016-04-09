// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include "amber/prelude/sampling.h"

namespace amber {
namespace rendering {

template <typename BaseSampler>
class PrimarySampleSpace
: public Sampler
{
public:
  using seed_type   = typename BaseSampler::seed_type;
  using result_type = typename BaseSampler::result_type;

  explicit PrimarySampleSpace(seed_type seed);

  result_type operator()();
  result_type operator[](std::size_t pos);
  void SetLargeStep();
  void Accept() noexcept;
  void Reject() noexcept;
  void ResetPosition() noexcept;

private:
  using timestamp_type = std::uint_fast64_t;
  struct Sample;

  BaseSampler sampler_;
  std::vector<Sample> samples_;
  std::vector<Sample> proposals_;
  timestamp_type last_large_step_time_;
  timestamp_type time_;
  bool large_step_;
  std::size_t pos_;

  Sample GenerateSample();
  Sample MutateSample(const Sample& sample);
  result_type Mutate(result_type value);
};

template <typename BaseSampler>
struct PrimarySampleSpace<BaseSampler>::Sample
{
  timestamp_type timestamp;
  result_type value;

  Sample() noexcept;
  Sample(timestamp_type timestamp, result_type value) noexcept;
};

template <typename BaseSampler>
class PrimarySampleSpacePair
{
public:
  using seed_type   = typename BaseSampler::seed_type;
  using result_type = typename BaseSampler::result_type;

  class Reference;

  explicit PrimarySampleSpacePair(seed_type seed);
  PrimarySampleSpacePair(const PrimarySampleSpacePair<BaseSampler>& pssp);

  PrimarySampleSpacePair<BaseSampler>&
  operator=(const PrimarySampleSpacePair<BaseSampler>& pssp);

  Reference& Light() noexcept { return light_; }
  Reference& Eye() noexcept { return eye_; }

  void SetLargeStep();
  void Accept() noexcept;
  void Reject() noexcept;
  void ResetPosition() noexcept;

private:
  PrimarySampleSpace<BaseSampler> pss_;
  Reference light_;
  Reference eye_;

  void ResetReference() noexcept;
};

template <typename BaseSampler>
class PrimarySampleSpacePair<BaseSampler>::Reference
: public Sampler
{
public:
  result_type operator()();

private:
  friend class PrimarySampleSpacePair;

  PrimarySampleSpace<BaseSampler>* pss_;
  std::size_t pos_;

  Reference(PrimarySampleSpace<BaseSampler>* pss, std::size_t pos) noexcept;
};



template <typename BaseSampler>
PrimarySampleSpace<BaseSampler>::PrimarySampleSpace(seed_type seed)
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
-> result_type
{
  const auto position = pos_++;

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
PrimarySampleSpace<BaseSampler>::operator[](std::size_t pos)
-> result_type
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
PrimarySampleSpace<BaseSampler>::MutateSample(const Sample& sample)
-> Sample
{
  return Sample(sample.timestamp + 1, Mutate(sample.value));
}

template <typename BaseSampler>
auto
PrimarySampleSpace<BaseSampler>::Mutate(result_type value)
-> result_type
{
  const auto s1 = static_cast<result_type>(1) / 1024;
  const auto s2 = static_cast<result_type>(1) / 64;
  const auto dv = s2 * std::exp(-std::log(s2 / s1) * sampler_());
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
  timestamp_type timestamp,
  result_type value
) noexcept
: timestamp(timestamp)
, value(value)
{}

template <typename BaseSampler>
PrimarySampleSpacePair<BaseSampler>::PrimarySampleSpacePair(seed_type seed)
: pss_(seed)
, light_(&pss_, 0)
, eye_(&pss_, 1)
{}

template <typename BaseSampler>
PrimarySampleSpacePair<BaseSampler>
::PrimarySampleSpacePair(const PrimarySampleSpacePair<BaseSampler>& pssp)
: pss_(pssp.pss_)
, light_(&pss_, 0)
, eye_(&pss_, 1)
{}

template <typename BaseSampler>
PrimarySampleSpacePair<BaseSampler>&
PrimarySampleSpacePair<BaseSampler>
::operator=(const PrimarySampleSpacePair<BaseSampler>& pssp)
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
  PrimarySampleSpace<BaseSampler>* pss,
  std::size_t pos
) noexcept
: pss_(pss)
, pos_(pos)
{}

template <typename BaseSampler>
auto
PrimarySampleSpacePair<BaseSampler>::Reference::operator()()
-> result_type
{
  const auto value = (*pss_)[pos_];
  pos_ += 2;
  return value;
}

}
}
