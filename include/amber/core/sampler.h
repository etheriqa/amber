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

#include <cmath>
#include <random>
#include <tuple>

#include "core/constant.h"
#include "core/vector3.h"

namespace amber {
namespace core {

class Sampler
{
public:
  using result_type = std::double_t;

  virtual result_type const operator()() = 0;
};

template <typename Engine = std::mt19937_64>
class DefaultSampler : public Sampler
{
public:
  using seed_type = typename Engine::result_type;

private:
  using typename Sampler::result_type;

  Engine engine_;

public:
  DefaultSampler() noexcept : engine_() {}

  explicit DefaultSampler(seed_type seed) noexcept : engine_(seed) {}

  Engine& engine() noexcept { return engine_; }

  result_type const operator()()
  {
    return std::uniform_real_distribution<result_type>(0, 1)(engine_);
  }
};

template <typename RealType>
RealType const
Uniform(Sampler& sampler)
{
  return static_cast<RealType>(sampler());
}

template <typename RealType>
RealType const
Uniform(RealType max, Sampler& sampler)
{
  return static_cast<RealType>(sampler()) * max;
}

template <typename RealType>
RealType const
Uniform(RealType min, RealType max, Sampler& sampler)
{
  return static_cast<RealType>(sampler()) * (max - min) + min;
}

template <typename RealType>
std::tuple<RealType const, RealType const>
Circle(Sampler& sampler)
{
  auto const theta = Uniform<RealType>(2 * kPI, sampler);
  return std::make_tuple(std::cos(theta), std::sin(theta));
}

/**
 * Sample a unit vector from the unit spherical surface uniformly
 * based on the ordinary *solid angle* measure.
 */
template <typename RealType>
Vector3<RealType> const
SphereSA(Sampler& sampler)
{
  auto const r0 = Uniform<RealType>(-1, 1, sampler);
  auto const r1 = Uniform<RealType>(sampler);
  auto const cos_theta = r0;
  auto const sin_theta = std::sqrt(1 - r0 * r0);
  auto const phi = 2 * static_cast<RealType>(kPI) * r1;
  return Vector3<RealType>(
    cos_theta * std::cos(phi),
    cos_theta * std::sin(phi),
    sin_theta
  );
}

/**
 * Sample a unit vector from the unit hemispherical surface uniformly
 * based on the ordinary *solid angle* measure.
 */
template <typename RealType>
std::tuple<Vector3<RealType> const, RealType const>
HemisphereSA(
  Vector3<RealType> const& u,
  Vector3<RealType> const& v,
  Vector3<RealType> const& w,
  Sampler& sampler
) {
  auto const r0 = Uniform<RealType>(sampler);
  auto const r1 = Uniform<RealType>(sampler);
  auto const cos_theta = r0;
  auto const sin_theta = std::sqrt(1 - r0 * r0);
  auto const phi = 2 * static_cast<RealType>(kPI) * r1;
  return std::make_tuple(
    u * sin_theta * std::cos(phi) +
    v * sin_theta * std::sin(phi) +
    w * cos_theta,
    cos_theta
  );
}

template <typename RealType>
std::tuple<Vector3<RealType> const, RealType const>
HemisphereSA(Vector3<RealType> const& w, Sampler& sampler)
{
  Vector3<RealType> u, v;
  std::tie(u, v) = OrthonormalBasis(w);
  return HemisphereSA(u, v, w, sampler);
}

/**
 * Sample a unit vector from the unit hemispherical surface uniformly
 * based on the *projected solid angle* measure (proportional to the cosine).
 */
template <typename RealType>
std::tuple<Vector3<RealType> const, RealType const>
HemispherePSA(
  Vector3<RealType> const& u,
  Vector3<RealType> const& v,
  Vector3<RealType> const& w,
  Sampler& sampler
) {
  auto const r0 = Uniform<RealType>(sampler);
  auto const r1 = Uniform<RealType>(sampler);
  auto const cos_theta = std::sqrt(r0);
  auto const sin_theta = std::sqrt(1 - r0);
  auto const phi = 2 * static_cast<RealType>(kPI) * r1;
  return std::make_tuple(
    u * sin_theta * std::cos(phi) +
    v * sin_theta * std::sin(phi) +
    w * cos_theta,
    cos_theta
  );
}

template <typename RealType>
std::tuple<Vector3<RealType> const, RealType const>
HemispherePSA(const Vector3<RealType>& w, Sampler& sampler)
{
  Vector3<RealType> u, v;
  std::tie(u, v) = OrthonormalBasis(w);
  return HemispherePSA(u, v, w, sampler);
}

template <typename RealType>
std::tuple<Vector3<RealType> const, RealType const>
CosinePower(
  Vector3<RealType> const& u,
  Vector3<RealType> const& v,
  Vector3<RealType> const& w,
  RealType exponent,
  Sampler& sampler
)
{
  auto const r0 = Uniform<RealType>(sampler);
  auto const r1 = Uniform<RealType>(sampler);
  auto const cos_theta = std::pow(r0, 1 / (exponent + 1));
  auto const sin_theta = std::sqrt(1 - cos_theta * cos_theta);
  auto const phi = 2 * static_cast<RealType>(kPI) * r1;
  return std::make_tuple(
    u * sin_theta * std::cos(phi) +
    v * sin_theta * std::sin(phi) +
    w * cos_theta,
    cos_theta
  );
}

template <typename RealType>
std::tuple<Vector3<RealType> const, RealType const>
CosinePower(Vector3<RealType> const& w, RealType exponent, Sampler& sampler)
{
  Vector3<RealType> u, v;
  std::tie(u, v) = OrthonormalBasis(w);
  return CosinePower(u, v, w, exponent, sampler);
}

}
}
