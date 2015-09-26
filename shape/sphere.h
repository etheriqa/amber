#pragma once

#include "algebra.h"
#include "constant.h"
#include "shape/shape.h"
#include "vector.h"

namespace amber {
namespace shape {

template <typename RealType>
class Sphere : public Shape<RealType>
{
public:
  using shape_type              = Shape<RealType>;

  using aabb_type               = typename shape_type::aabb_type;
  using hit_type                = typename shape_type::hit_type;
  using initial_ray_sample_type = typename shape_type::initial_ray_sample_type;
  using ray_type                = typename shape_type::ray_type;
  using real_type               = typename shape_type::real_type;

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
    const auto a = norm2(ray.direction);
    const auto b = -2 * dot(m_center - ray.origin, ray.direction);
    const auto c = norm2(m_center - ray.origin) - m_radius * m_radius;

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
