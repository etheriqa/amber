/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "algebra.h"
#include "constant.h"
#include "geometry/primitive/circle.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Cylinder : public Primitive<RealType>
{
public:
  using primitive_type          = Primitive<RealType>;

  using aabb_type               = typename primitive_type::aabb_type;
  using hit_type                = typename primitive_type::hit_type;
  using initial_ray_sample_type = typename primitive_type::initial_ray_sample_type;
  using ray_type                = typename primitive_type::ray_type;
  using real_type               = typename primitive_type::real_type;

  using circle_type             = Circle<real_type>;
  using vector3_type            = Vector3<real_type>;

private:
  vector3_type m_center, m_normal;
  real_type m_radius, m_height;

public:
  Cylinder(const vector3_type& center, const vector3_type& normal, real_type radius, real_type height) :
    m_center(center), m_normal(normalize(normal)), m_radius(radius), m_height(height) {}

  real_type surface_area() const noexcept
  {
    return 2 * static_cast<real_type>(kPI) * m_radius * m_height;
  }

  aabb_type aabb() const noexcept
  {
    const auto bottom = circle_type(m_center, m_normal, m_radius);
    const auto top = circle_type(m_center + m_height * m_normal, m_normal, m_radius);
    return bottom.aabb() + top.aabb();
  }

  hit_type intersect(const ray_type& ray) const noexcept
  {
    const auto OC = m_center - ray.origin;
    const auto u = ray.direction - dot(ray.direction, m_normal) * m_normal;
    const auto v = OC - dot(OC, m_normal) * m_normal;

    const auto a = u.squaredLength();
    const auto b = -2 * dot(u, v);
    const auto c = v.squaredLength() - m_radius * m_radius;

    bool hit;
    real_type alpha, beta;
    std::tie(hit, alpha, beta) = solve_quadratic(a, b, c);

    if (!hit) {
      return hit_type();
    }

    if (alpha > kEPS) {
      const auto h = dot(alpha * ray.direction - OC, m_normal);
      if (h >= 0 && h <= m_height) {
        return hit_type(
          ray.origin + alpha * ray.direction,
          ray.origin + alpha * ray.direction - m_center - h * m_normal,
          alpha
        );
      }
    }

    if (beta > kEPS) {
      const auto h = dot(beta * ray.direction - OC, m_normal);
      if (h >= 0 && h <= m_height) {
        return hit_type(
          ray.origin + beta * ray.direction,
          ray.origin + beta * ray.direction - m_center - h * m_normal,
          beta
        );
      }
    }

    return hit_type();
  }

  initial_ray_sample_type sample_initial_ray(Random& random) const
  {
    const auto height = random.uniform(m_height);

    vector3_type u, v;
    std::tie(u, v) = orthonormal_basis(m_normal);

    real_type x, y;
    std::tie(x, y) = random.circle<real_type>();

    const auto normal = u * x + v * y;

    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = random.hemisphere_psa(normal);

    return initial_ray_sample_type(
      ray_type(m_center + m_normal * height + normal * m_radius, direction_o),
      normal
    );
  }
};

}
}
}
