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

#include "amber/rendering/bidirectional_path_sampling.h"

namespace amber {
namespace rendering {

/** Buffer object for the purpose of efficient calculation of MIS weight.
 */
template <typename Radiant>
class UnifiedPathSamplingBuffer
: private BidirectionalPathSamplingBuffer<Radiant>
{
public:
  using typename BidirectionalPathSamplingBuffer<Radiant>::SubpathConstIterator;

  /** Constructor.
   */
  UnifiedPathSamplingBuffer() noexcept;

  /** Buffers full path information.
   */
  void
  Buffer(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    SubpathConstIterator light_first,
    SubpathConstIterator light_last,
    SubpathConstIterator eye_first,
    SubpathConstIterator eye_last,
    real_type kernel,
    bool did_virtual_perturbation
  );

  /** Calculates MIS weight for the current buffer.
   */
  real_type MISWeight(const MIS& mis) const noexcept;

private:
  using BidirectionalPathSamplingBuffer<Radiant>::geometry_factor_;
  using BidirectionalPathSamplingBuffer<Radiant>::p_light_;
  using BidirectionalPathSamplingBuffer<Radiant>::p_importance_;
  using BidirectionalPathSamplingBuffer<Radiant>::p_technique_;

  void
  ExtendTechniqueProbability(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    SubpathConstIterator light_first,
    SubpathConstIterator light_last,
    SubpathConstIterator eye_first,
    SubpathConstIterator eye_last,
    real_type kernel,
    bool did_virtual_perturbation
  );
};





template <typename Radiant>
UnifiedPathSamplingBuffer<Radiant>::UnifiedPathSamplingBuffer() noexcept
: BidirectionalPathSamplingBuffer<Radiant>()
{}

template <typename Radiant>
void
UnifiedPathSamplingBuffer<Radiant>::Buffer(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  SubpathConstIterator light_first,
  SubpathConstIterator light_last,
  SubpathConstIterator eye_first,
  SubpathConstIterator eye_last,
  real_type kernel,
  bool did_virtual_perturbation
)
{
  if (!did_virtual_perturbation) {
    light_last--;
  }

  BidirectionalPathSamplingBuffer<Radiant>::BufferGeometryFactor(
    scene, sensor, light_first, light_last, eye_first, eye_last);
  BidirectionalPathSamplingBuffer<Radiant>::BufferLightProbability(
    scene, sensor, light_first, light_last, eye_first, eye_last);
  BidirectionalPathSamplingBuffer<Radiant>::BufferImportanceProbability(
    scene, sensor, light_first, light_last, eye_first, eye_last);
  BidirectionalPathSamplingBuffer<Radiant>::BufferTechniqueProbability(
    scene, sensor, light_first, light_last, eye_first, eye_last);

  ExtendTechniqueProbability(
    scene,
    sensor,
    light_first,
    light_last,
    eye_first,
    eye_last,
    kernel,
    did_virtual_perturbation
  );
}

template <typename Radiant>
real_type
UnifiedPathSamplingBuffer<Radiant>::MISWeight(const MIS& mis) const noexcept
{
  const auto weight = BidirectionalPathSamplingBuffer<Radiant>::MISWeight(mis);
  return std::isfinite(weight) ? weight : 0;
}

template <typename Radiant>
void
UnifiedPathSamplingBuffer<Radiant>::ExtendTechniqueProbability(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  SubpathConstIterator light_first,
  SubpathConstIterator light_last,
  SubpathConstIterator eye_first,
  SubpathConstIterator eye_last,
  real_type kernel,
  bool did_virtual_perturbation
)
{
  const size_t s = std::distance(light_first, light_last) + 1;
  const size_t t = std::distance(eye_first, eye_last);
  const auto k = s + t - 2;
  const auto n_mc_techniques = k + 2;
  const auto n_de_techniques = k - 1;
  const auto n_techniques = n_mc_techniques + n_de_techniques;

  std::copy_n(
    p_technique_.begin() + 1,
    n_de_techniques,
    std::back_inserter(p_technique_)
  );

  if (did_virtual_perturbation) {
    std::for_each(
      p_technique_.end() - n_de_techniques,
      p_technique_.end(),
      [&](auto& p){ p *= sensor.Size(); }
    );
  } else {
    std::for_each(
      p_technique_.begin(),
      p_technique_.begin() + n_mc_techniques,
      [&](auto& p){ p /= sensor.Size(); }
    );
  }

  if (did_virtual_perturbation) {
    std::for_each(
      p_technique_.end() - n_de_techniques,
      p_technique_.end(),
      [&](auto& p){ p /= kernel; }
    );
  } else {
    std::for_each(
      p_technique_.begin(),
      p_technique_.begin() + n_mc_techniques,
      [&](auto& p){ p *= kernel; }
    );
  }

  for (size_t i = 0; i < n_de_techniques; i++) {
    const auto p_x_i = p_importance_[i] * geometry_factor_[i];
    if (did_virtual_perturbation || s != i + 2) {
      p_technique_[n_mc_techniques + i] *= p_x_i;
    } else {
      for (size_t j = 0; j < n_techniques; j++) {
        if (j == n_mc_techniques + i) {
          continue;
        } else if (p_x_i == 0) {
          p_technique_[j] = std::numeric_limits<real_type>::infinity();
        } else {
          p_technique_[j] /= p_x_i;
        }
      }
    }
  }

  for (path_size_type i = 1; i + 1 + did_virtual_perturbation < s; i++) {
    if (scene.Surface(light_first[i].Object()) != SurfaceType::Diffuse) {
      p_technique_[i] = 0;
      p_technique_[i + 1] = 0;
      p_technique_[n_mc_techniques + i - 1] = 0;
    }
  }

  for (path_size_type i = 1; i + 1 < t; i++) {
    if (scene.Surface(eye_first[i].Object()) != SurfaceType::Diffuse) {
      p_technique_[n_mc_techniques - i - 1] = 0;
      p_technique_[n_mc_techniques - i - 2] = 0;
      p_technique_[n_techniques - i] = 0;
    }
  }
}

}
}
