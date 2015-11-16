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

class Sampler {
protected:
  using real_type = std::double_t;

public:
  virtual real_type canonical() = 0;

  template <typename RealType>
  RealType
  uniform() {
    return static_cast<RealType>(canonical());
  }

  template <typename RealType>
  RealType
  uniform(RealType max) {
    return static_cast<RealType>(canonical() * max);
  }

  template <typename RealType>
  RealType
  uniform(RealType min, RealType max) {
    return static_cast<RealType>(min + canonical() * (max - min));
  }

  template <typename RealType>
  std::tuple<RealType, RealType>
  circle() {
    const auto theta = uniform<RealType>(2 * kPI);
    return std::make_tuple(std::cos(theta), std::sin(theta));
  }

  /**
   * Sample a unit vector from the unit spherical surface uniformly
   * based on the ordinary *solid angle* measure.
   */
  template <typename RealType>
  Vector3<RealType>
  sphereSA() {
    const auto r0 = uniform<RealType>(-1, 1), r1 = uniform<RealType>();
    const auto cos_theta = r0;
    const auto sin_theta = std::sqrt(1 - r0 * r0);
    const auto phi = 2 * static_cast<RealType>(kPI) * r1;
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
  std::tuple<Vector3<RealType>, RealType>
  hemisphereSA(const Vector3<RealType>& u,
               const Vector3<RealType>& v,
               const Vector3<RealType>& w) {
    const auto r0 = uniform<RealType>(), r1 = uniform<RealType>();
    const auto cos_theta = r0;
    const auto sin_theta = std::sqrt(1 - r0 * r0);
    const auto phi = 2 * static_cast<RealType>(kPI) * r1;
    return std::make_tuple(
      u * sin_theta * std::cos(phi) +
        v * sin_theta * std::sin(phi) +
        w * cos_theta,
      cos_theta
    );
  }

  template <typename RealType>
  std::tuple<Vector3<RealType>, RealType>
  hemisphereSA(const Vector3<RealType>& w) {
    Vector3<RealType> u, v;
    std::tie(u, v) = OrthonormalBasis(w);
    return hemisphereSA(u, v, w);
  }

  /**
   * Sample a unit vector from the unit hemispherical surface uniformly
   * based on the *projected solid angle* measure (proportional to the cosine).
   */
  template <typename RealType>
  std::tuple<Vector3<RealType>, RealType>
  hemispherePSA(const Vector3<RealType>& u,
                const Vector3<RealType>& v,
                const Vector3<RealType>& w) {
    const auto r0 = uniform<RealType>(), r1 = uniform<RealType>();
    const auto cos_theta = std::sqrt(r0);
    const auto sin_theta = std::sqrt(1 - r0);
    const auto phi = 2 * static_cast<RealType>(kPI) * r1;
    return std::make_tuple(
      u * sin_theta * std::cos(phi) + v * sin_theta * std::sin(phi) + w * cos_theta,
      cos_theta
    );
  }

  template <typename RealType>
  std::tuple<Vector3<RealType>, RealType>
  hemispherePSA(const Vector3<RealType>& w) {
    Vector3<RealType> u, v;
    std::tie(u, v) = OrthonormalBasis(w);
    return hemispherePSA(u, v, w);
  }
};

template <typename Engine = std::mt19937_64>
class DefaultSampler : public Sampler {
public:
  using result_type = typename Engine::result_type;

private:
  using real_type   = typename Sampler::real_type;

  Engine engine_;

public:
  DefaultSampler() noexcept : engine_() {}

  explicit DefaultSampler(result_type seed) noexcept : engine_(seed) {}

  real_type canonical() {
    return std::uniform_real_distribution<real_type>(0, 1)(engine_);
  }

  result_type operator()() {
    return engine_();
  }
};

}
}
