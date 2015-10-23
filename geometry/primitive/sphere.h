#pragma once

#include "algebra.h"
#include "constant.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Sphere : public Primitive<RealType>
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
  vector3_type m_center;
  real_type m_radius;

public:
  Sphere(const vector3_type& center, real_type radius) :
    m_center(center), m_radius(radius) {}

  real_type surface_area() const noexcept
  {
    return 4 * static_cast<real_type>(kPI) * m_radius * m_radius;
  }

  aabb_type aabb() const noexcept
  {
    return aabb_type(m_center - m_radius, m_center + m_radius);
  }

  hit_type intersect(const ray_type& ray) const noexcept
  {
    const real_type a = 1;
    const auto b = -2 * dot(m_center - ray.origin, ray.direction);
    const auto c = (m_center - ray.origin).squaredLength() - m_radius * m_radius;

    bool hit;
    real_type alpha, beta;
    std::tie(hit, alpha, beta) = solve_quadratic(a, b, c);

    if (!hit) {
      return hit_type();
    }

    if (alpha > kEPS) {
      return hit_type(
        ray.origin + alpha * ray.direction,
        ray.origin + alpha * ray.direction - m_center,
        alpha
      );
    }

    if (beta > kEPS) {
      return hit_type(
        ray.origin + beta * ray.direction,
        ray.origin + beta * ray.direction - m_center,
        beta
      );
    }

    return hit_type();
  }

  initial_ray_sample_type sample_initial_ray(Random& random) const
  {
    const auto normal = random.sphere_sa<real_type>();

    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = random.hemisphere_psa(normal);

    return initial_ray_sample_type(
      ray_type(m_center + m_radius * normal, direction_o),
      normal
    );
  }
};

}
}
}
