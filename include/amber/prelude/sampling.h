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

#include <cmath>
#include <random>
#include <tuple>
#include <type_traits>

#include "amber/constants.h"
#include "amber/prelude/vector2.h"
#include "amber/prelude/vector3.h"

namespace amber {
namespace prelude {

class Sampler
{
public:
  using result_type = std::double_t;

  virtual const result_type operator()() = 0;
};

template <typename Engine>
class GenericSampler
: public Sampler
{
public:
  using typename Sampler::result_type;
  using seed_type = typename Engine::result_type;

  explicit GenericSampler(seed_type seed);

  const result_type operator()();

private:
  Engine engine_;
};

template <typename T>
const T
Uniform(Sampler& sampler);

template <typename T>
const T
Uniform(const T max, Sampler& sampler);

template <typename T>
const T
Uniform(const T min, const T max, Sampler& sampler);

template <typename T>
const Vector2<T>
Circle(Sampler& sampler);

/**
 * Sample a unit vector from the unit spherical surface uniformly
 * based on the ordinary *solid angle* measure.
 */
template <typename T>
const UnitVector3<T>
SphereSA(Sampler& sampler);

/**
 * Sample a unit vector from the unit hemispherical surface uniformly
 * based on the ordinary *solid angle* measure.
 */
template <typename T>
const UnitVector3<T>
HemisphereSA(
  const UnitVector3<T>& u,
  const UnitVector3<T>& v,
  const UnitVector3<T>& w,
  Sampler& sampler
);

template <typename T>
const UnitVector3<T>
HemisphereSA(
  const UnitVector3<T>& w,
  Sampler& sampler
);

/**
 * Sample a unit vector from the unit hemispherical surface uniformly
 * based on the *projected solid angle* measure (proportional to the cosine).
 */
template <typename T>
const UnitVector3<T>
HemispherePSA(
  const UnitVector3<T>& u,
  const UnitVector3<T>& v,
  const UnitVector3<T>& w,
  Sampler& sampler
);

template <typename T>
const UnitVector3<T>
HemispherePSA(
  const UnitVector3<T>& w,
  Sampler& sampler
);

template <typename T>
const UnitVector3<T>
CosinePower(
  const UnitVector3<T>& u,
  const UnitVector3<T>& v,
  const UnitVector3<T>& w,
  const T exponent,
  Sampler& sampler
);

template <typename T>
const UnitVector3<T>
CosinePower(
  const UnitVector3<T>& u,
  const T exponent,
  Sampler& sampler
);



template <typename Engine>
GenericSampler<Engine>::GenericSampler(seed_type seed)
: engine_(seed)
{}

template <typename Engine>
auto
GenericSampler<Engine>::operator()()
-> const result_type
{
  return std::uniform_real_distribution<result_type>(0, 1)(engine_);
}

template <typename T>
const T
Uniform(Sampler& sampler)
{
  return static_cast<T>(sampler());
}

template <typename T>
const T
Uniform(const T max, Sampler& sampler)
{
  return static_cast<T>(sampler()) * max;
}

template <typename T>
const T
Uniform(const T min, const T max, Sampler& sampler)
{
  return static_cast<T>(sampler()) * (max - min) + min;
}

template <typename T>
const Vector2<T>
Circle(Sampler& sampler)
{
  const auto theta = Uniform<T>(2 * kPI, sampler);
  return Vector2<T>(std::cos(theta), std::sin(theta));
}

template <typename T>
const UnitVector3<T>
SphereSA(Sampler& sampler)
{
  const auto r0 = Uniform<T>(-1, 1, sampler);
  const auto r1 = Uniform<T>(sampler);
  const auto cos_theta = r0;
  const auto sin_theta = std::sqrt(1 - r0 * r0);
  const auto phi = 2 * static_cast<T>(kPI) * r1;
  return UnitVector3<T>(
    cos_theta * std::cos(phi),
    cos_theta * std::sin(phi),
    sin_theta
  );
}

template <typename T>
const UnitVector3<T>
HemisphereSA(
  const UnitVector3<T>& u,
  const UnitVector3<T>& v,
  const UnitVector3<T>& w,
  Sampler& sampler
)
{
  const auto r0 = Uniform<T>(sampler);
  const auto r1 = Uniform<T>(sampler);
  const auto cos_theta = r0;
  const auto sin_theta = std::sqrt(1 - r0 * r0);
  const auto phi = 2 * static_cast<T>(kPI) * r1;
  return UnitVector3<T>(
    u * sin_theta * std::cos(phi) +
    v * sin_theta * std::sin(phi) +
    w * cos_theta
  );
}

template <typename T>
const UnitVector3<T>
HemisphereSA(
  const UnitVector3<T>& w,
  Sampler& sampler
)
{
  UnitVector3<T> u, v;
  std::tie(u, v) = OrthonormalBasis(w);
  return HemisphereSA(u, v, w, sampler);
}

template <typename T>
const UnitVector3<T>
HemispherePSA(
  const UnitVector3<T>& u,
  const UnitVector3<T>& v,
  const UnitVector3<T>& w,
  Sampler& sampler
)
{
  const auto r0 = Uniform<T>(sampler);
  const auto r1 = Uniform<T>(sampler);
  const auto cos_theta = std::sqrt(r0);
  const auto sin_theta = std::sqrt(1 - r0);
  const auto phi = 2 * static_cast<T>(kPI) * r1;
  return UnitVector3<T>(
    u * sin_theta * std::cos(phi) +
    v * sin_theta * std::sin(phi) +
    w * cos_theta
  );
}

template <typename T>
const UnitVector3<T>
HemispherePSA(
  const UnitVector3<T>& w,
  Sampler& sampler
)
{
  UnitVector3<T> u, v;
  std::tie(u, v) = OrthonormalBasis(w);
  return HemispherePSA(u, v, w, sampler);
}

template <typename T>
const UnitVector3<T>
CosinePower(
  const UnitVector3<T>& u,
  const UnitVector3<T>& v,
  const UnitVector3<T>& w,
  const T exponent,
  Sampler& sampler
)
{
  const auto r0 = Uniform<T>(sampler);
  const auto r1 = Uniform<T>(sampler);
  const auto cos_theta = std::pow(r0, 1 / (exponent + 1));
  const auto sin_theta = std::sqrt(1 - cos_theta * cos_theta);
  const auto phi = 2 * static_cast<T>(kPI) * r1;
  return UnitVector3<T>(
    u * sin_theta * std::cos(phi) +
    v * sin_theta * std::sin(phi) +
    w * cos_theta
  );
}

template <typename T>
const UnitVector3<T>
CosinePower(
  const UnitVector3<T>& w,
  const T exponent,
  Sampler& sampler
)
{
  UnitVector3<T> u, v;
  std::tie(u, v) = OrthonormalBasis(w);
  return CosinePower(u, v, w, exponent, sampler);
}

}
}
