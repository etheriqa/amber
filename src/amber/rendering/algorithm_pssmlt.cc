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

#include "amber/prelude/accumulator.h"
#include "amber/prelude/hit.h"
#include "amber/prelude/image.h"
#include "amber/prelude/sampling.h"
#include "amber/prelude/vector3.h"
#include "amber/rendering/algorithm.h"
#include "amber/rendering/algorithm_pssmlt.h"
#include "amber/rendering/bidirectional_path_sampling.h"
#include "amber/rendering/context.h"
#include "amber/rendering/multiple_importance_sampling.h"
#include "amber/rendering/parallel.h"
#include "amber/rendering/primary_sample_space.h"

namespace amber {
namespace rendering {

template <typename Radiant>
class PrimarySampleSpaceMLT
: public Algorithm<Radiant>
{
public:
  PrimarySampleSpaceMLT(
    std::size_t n_discard_samples,
    real_type p_large_step
  ) noexcept;

  const Image<Radiant>
  Render(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    Context& context
  );

private:
  class Thread;
  class State;
  class Chain;
  using NormalizationFactor = Accumulator<real_type>;

  std::size_t n_discard_samples_;
  real_type p_large_step_;
};

template <typename Radiant>
class PrimarySampleSpaceMLT<Radiant>::Thread
{
public:
  Thread(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    std::size_t n_discard_samples,
    real_type p_large_step,
    NormalizationFactor& b
  );
  Thread(const Thread& thread);

  const Image<Radiant> operator()();

private:
  const Scene<Radiant>& scene_;
  const Sensor& sensor_;
  std::size_t n_discard_samples_;
  real_type p_large_step_;
  NormalizationFactor& b_;
  MTSampler sampler_;
  Chain chain_;
  Subpath<Radiant> light_path_;
  Subpath<Radiant> eye_path_;
  BidirectionalPathSamplingBuffer<Radiant> bdps_buffer_;

  State Step(real_type b, NormalizationFactor& b_buffer);
  State ProposeState();
};

template <typename Radiant>
class PrimarySampleSpaceMLT<Radiant>::State
{
public:
  State() noexcept;
  State(
    PixelValue<Radiant>&& eye_image,
    SparseImage<Radiant>&& light_image
  ) noexcept;

  const real_type Contribution() const noexcept { return contribution_; }
  void AddWeight(real_type weight) noexcept { weight_ += weight; }
  void Render(Image<Radiant>& image) noexcept;

private:
  PixelValue<Radiant> eye_image_;
  SparseImage<Radiant> light_image_;
  real_type contribution_;
  real_type weight_;
};

template <typename Radiant>
class PrimarySampleSpaceMLT<Radiant>::Chain
: public State
, public MTPrimarySampleSpacePair
{
public:
  using MTPrimarySampleSpacePair::seed_type;

  explicit Chain(seed_type seed);
};



std::unique_ptr<Algorithm<RGB>>
MakeRGBPrimarySampleSpaceMLT(
  std::size_t n_discard_samples,
  real_type p_large_step
)
{
  return std::make_unique<PrimarySampleSpaceMLT<RGB>>(
    n_discard_samples,
    p_large_step
  );
}

template <typename Radiant>
PrimarySampleSpaceMLT<Radiant>::PrimarySampleSpaceMLT(
  std::size_t n_discard_samples,
  real_type p_large_step
) noexcept
: n_discard_samples_(n_discard_samples)
, p_large_step_(p_large_step)
{}

template <typename Radiant>
const Image<Radiant>
PrimarySampleSpaceMLT<Radiant>::Render(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  Context& context
)
{
  NormalizationFactor b(0);

  return ParallelMean<Image<Radiant>>(
    context,
    sensor.CreateImage<Radiant>(),
    Thread(scene, sensor, n_discard_samples_, p_large_step_, b)
  );
}

template <typename Radiant>
PrimarySampleSpaceMLT<Radiant>::Thread::Thread(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  std::size_t n_discard_samples,
  real_type p_large_step,
  NormalizationFactor& b
)
: scene_(scene)
, sensor_(sensor)
, n_discard_samples_(n_discard_samples)
, p_large_step_(p_large_step)
, b_(b)
, sampler_(std::random_device()())
, chain_(std::random_device()())
, light_path_()
, eye_path_()
, bdps_buffer_()
{}

template <typename Radiant>
PrimarySampleSpaceMLT<Radiant>::Thread::Thread(const Thread& thread)
: Thread(
    thread.scene_,
    thread.sensor_,
    thread.n_discard_samples_,
    thread.p_large_step_,
    thread.b_
  )
{}

template <typename Radiant>
const Image<Radiant>
PrimarySampleSpaceMLT<Radiant>::Thread::operator()()
{
  if (chain_.Contribution() == static_cast<real_type>(0)) {
    NormalizationFactor b_buffer(0);
    for (pixel_size_type i = 0; i < n_discard_samples_; i++) {
      Step(1, b_buffer);
    }
    b_ += std::move(b_buffer);
  }

  auto image = sensor_.CreateImage<Radiant>();

  {
    const auto b_snapshot = b_.Mean();
    NormalizationFactor b_buffer(0);
    for (pixel_size_type i = 0; i < image.Size(); i++) {
      Step(b_snapshot, b_buffer).Render(image);
    }
    chain_.Render(image);
    b_ += std::move(b_buffer);
  }

  return image;
}

template <typename Radiant>
auto
PrimarySampleSpaceMLT<Radiant>::Thread::Step(
  real_type b,
  NormalizationFactor& b_buffer
)
-> State
{
  const auto large_step = prelude::Uniform<real_type>(sampler_) < p_large_step_;
  if (large_step) {
    chain_.SetLargeStep();
  }

  auto proposal = ProposeState();
  if (large_step) {
    b_buffer += proposal.Contribution();
  }

  const auto p_acceptance =
    std::min<real_type>(1, proposal.Contribution() / chain_.Contribution());

  proposal.AddWeight(
    (p_acceptance + large_step) / (proposal.Contribution() / b + p_large_step_)
  );
  chain_.AddWeight(
    (1 - p_acceptance) / (chain_.Contribution() / b + p_large_step_)
  );

  if (prelude::Uniform<real_type>(sampler_) < p_acceptance) {
    // accepted
    std::swap<State>(chain_, proposal);
    chain_.Accept();
  } else {
    // rejected
    chain_.Reject();
  }

  return proposal;
}

template <typename Radiant>
auto
PrimarySampleSpaceMLT<Radiant>::Thread::ProposeState()
-> State
{
  GenerateLightSubpath(scene_, chain_.Light(), light_path_);

  const auto pixel =
    GenerateEyeSubpath(scene_, sensor_, chain_.Eye(), eye_path_);

  SparseImage<Radiant> light_image;
  const auto measurement = Combine(
    scene_,
    sensor_,
    light_path_,
    eye_path_,
    PowerHeuristic<real_type>(),
    bdps_buffer_,
    light_image
  );

  return State(PixelValue<Radiant>(pixel, measurement), std::move(light_image));
}

template <typename Radiant>
PrimarySampleSpaceMLT<Radiant>::State::State() noexcept
: eye_image_()
, light_image_()
, contribution_(0)
, weight_(0)
{}

template <typename Radiant>
PrimarySampleSpaceMLT<Radiant>::State::State(
  PixelValue<Radiant>&& eye_image,
  SparseImage<Radiant>&& light_image
) noexcept
: eye_image_(std::move(eye_image))
, light_image_(std::move(light_image))
, contribution_(0)
, weight_(0)
{
  contribution_ += Sum(eye_image_.Value());
  for (const auto& measurement : light_image_) {
    contribution_ += Sum(measurement.Value());
  }
}

template <typename Radiant>
void
PrimarySampleSpaceMLT<Radiant>::State::Render(Image<Radiant>& image) noexcept
{
  image[eye_image_.Pixel()] += eye_image_.Value() * weight_;
  image += light_image_ * weight_;
  weight_ = 0;
}

template <typename Radiant>
PrimarySampleSpaceMLT<Radiant>::Chain::Chain(seed_type seed)
: State()
, MTPrimarySampleSpacePair(seed)
{}

}
}
