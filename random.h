#pragma once

#include <random>
#include <tuple>
#include "constant.h"
#include "geometry/vector.h"

namespace amber {

class Random {
private:
  std::mt19937_64 m_engine;

public:
  Random() {}

  auto& generator() {
    return m_engine;
  }

  template <typename RealType>
  RealType
  uniform() {
    return uniform<RealType>(0, 1);
  }

  template <typename RealType>
  RealType
  uniform(RealType max) {
    return uniform<RealType>(0, max);
  }

  template <typename RealType>
  RealType
  uniform(RealType min, RealType max) {
    return std::uniform_real_distribution<RealType>(min, max)(m_engine);
  }

  template <typename RealType>
  std::tuple<RealType, RealType>
  circle() {
    const auto theta = uniform(static_cast<RealType>(2 * kPI));
    return std::make_tuple(std::cos(theta), std::sin(theta));
  }

  /**
   * Sample a unit vector from the unit spherical surface uniformly based on the ordinary *solid angle* measure.
   */
  template <typename RealType>
  geometry::Vector3<RealType>
  sphere_sa() {
    const auto r0 = uniform<RealType>(-1, 1), r1 = uniform<RealType>();
    const auto cos_theta = r0;
    const auto sin_theta = std::sqrt(1 - r0 * r0);
    const auto phi = 2 * static_cast<RealType>(kPI) * r1;
    return geometry::Vector3<RealType>(
      cos_theta * std::cos(phi),
      cos_theta * std::sin(phi),
      sin_theta
    );
  }

  /**
   * Sample a unit vector from the unit hemispherical surface uniformly based on the ordinary *solid angle* measure.
   */
  template <typename RealType>
  std::tuple<geometry::Vector3<RealType>, RealType>
  hemisphere_sa(const geometry::Vector3<RealType>& u,
                const geometry::Vector3<RealType>& v,
                const geometry::Vector3<RealType>& w) {
    const auto r0 = uniform<RealType>(), r1 = uniform<RealType>();
    const auto cos_theta = r0;
    const auto sin_theta = std::sqrt(1 - r0 * r0);
    const auto phi = static_cast<RealType>(2 * kPI) * r1;
    return std::make_tuple(
      u * sin_theta * std::cos(phi) + v * sin_theta * std::sin(phi) + w * cos_theta,
      cos_theta
    );
  }

  template <typename RealType>
  std::tuple<geometry::Vector3<RealType>, RealType>
  hemisphere_sa(const geometry::Vector3<RealType>& w) {
    geometry::Vector3<RealType> u, v;
    std::tie(u, v) = orthonormal_basis(w);
    return hemisphere_sa(u, v, w);
  }

  /**
   * Sample a unit vector from the unit hemispherical surface uniformly based on the *projected solid angle* measure.
   */
  template <typename RealType>
  std::tuple<geometry::Vector3<RealType>, RealType>
  hemisphere_psa(const geometry::Vector3<RealType>& u,
                 const geometry::Vector3<RealType>& v,
                 const geometry::Vector3<RealType>& w) {
    const auto r0 = uniform<RealType>(), r1 = uniform<RealType>();
    const auto cos_theta = std::sqrt(r0);
    const auto sin_theta = std::sqrt(1 - r0);
    const auto phi = static_cast<RealType>(2 * kPI) * r1;
    return std::make_tuple(
      u * sin_theta * std::cos(phi) + v * sin_theta * std::sin(phi) + w * cos_theta,
      cos_theta
    );
  }

  template <typename RealType>
  std::tuple<geometry::Vector3<RealType>, RealType>
  hemisphere_psa(const geometry::Vector3<RealType>& w) {
    geometry::Vector3<RealType> u, v;
    std::tie(u, v) = orthonormal_basis(w);
    return hemisphere_psa(u, v, w);
  }
};

}
