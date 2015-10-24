/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include "constant.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Triangle : public Primitive<RealType>
{
public:
  using primitive_type          = Primitive<RealType>;

  using aabb_type               = typename primitive_type::aabb_type;
  using hit_type                = typename primitive_type::hit_type;
  using initial_ray_sample_type = typename primitive_type::initial_ray_sample_type;
  using ray_type                = typename primitive_type::ray_type;
  using real_type               = typename primitive_type::real_type;

  using vector3_type            = Vector3<real_type>;

private:
  vector3_type m_v0, m_v1, m_v2;
  vector3_type m_normal;

public:
  Triangle(const vector3_type& v0, const vector3_type& v1, const vector3_type& v2) :
    m_v0(v0), m_v1(v1), m_v2(v2)
  {
    m_normal = normalize(cross(m_v1 - m_v0, m_v2 - m_v0));
  }

  real_type surface_area() const noexcept
  {
    return (cross(m_v1 - m_v0, m_v2 - m_v0)).length() / 2;
  }

  aabb_type aabb() const noexcept
  {
    return aabb_type(
      vector3_type(
        std::min({m_v0.x(), m_v1.x(), m_v2.x()}),
        std::min({m_v0.y(), m_v1.y(), m_v2.y()}),
        std::min({m_v0.z(), m_v1.z(), m_v2.z()})
      ),
      vector3_type(
        std::max({m_v0.x(), m_v1.x(), m_v2.x()}),
        std::max({m_v0.y(), m_v1.y(), m_v2.y()}),
        std::max({m_v0.z(), m_v1.z(), m_v2.z()})
      )
    );
  }

  hit_type intersect(const ray_type& ray) const noexcept
  {
    const auto E1 = m_v1 - m_v0;
    const auto E2 = m_v2 - m_v0;

    const auto P = cross(ray.direction, E2);
    const auto det = dot(P, E1);

    const auto T = ray.origin - m_v0;
    const auto u = dot(P, T) / det;
    if (u > 1 || u < 0) {
      return hit_type();
    }

    const auto Q = cross(T, E1);
    const auto v = dot(Q, ray.direction) / det;
    if (v > 1 || v < 0) {
      return hit_type();
    }

    if (u + v > 1) {
      return hit_type();
    }

    const auto t = dot(Q, E2) / det;
    if (t < kEPS) {
      return hit_type();
    }

    return hit_type(
      m_v0 + u * E1 + v * E2,
      m_normal,
      t
    );
  }

  initial_ray_sample_type sample_initial_ray(Random& random) const
  {
    auto u = random.uniform<real_type>(), v = random.uniform<real_type>();

    if (u + v >= 1) {
      u = 1 - u;
      v = 1 - v;
    }

    const auto origin = (1 - u - v) * m_v0 + u * m_v1 + v * m_v2;

    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = random.hemisphere_psa(m_normal);

    return initial_ray_sample_type(
      ray_type(origin, direction_o),
      m_normal
    );
  }
};

}
}
}
