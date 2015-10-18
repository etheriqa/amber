#pragma once

#include <cmath>
#include "constant.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Disk : public Primitive<RealType>
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
  vector3_type m_center, m_normal;
  real_type m_radius;

public:
  Circle(const vector3_type& center, const vector3_type& normal, real_type radius) :
    m_center(center), m_normal(normalize(normal)), m_radius(radius) {}

  real_type surface_area() const noexcept
  {
    return static_cast<real_type>(kPI) * m_radius * m_radius;
  }

  aabb_type aabb() const noexcept
  {
    const auto factor = vector3_type(
      std::sqrt(1 - m_normal.x * m_normal.x),
      std::sqrt(1 - m_normal.y * m_normal.y),
      std::sqrt(1 - m_normal.z * m_normal.z)
    );

    return aabb_type(m_center - m_radius * factor, m_center + m_radius * factor);
  }

  hit_type intersect(const ray_type& ray) const noexcept
  {
     const auto cos_theta = dot(ray.direction, m_normal);
     if (cos_theta == 0) {
       return hit_type();
     }

     const auto t = dot(m_center - ray.origin, m_normal) / cos_theta;
     if (t < kEPS) {
       return hit_type();
     }

     if (norm2(ray.origin + t * ray.direction - m_center) > m_radius * m_radius) {
       return hit_type();
     }

     return hit_type(
      ray.origin + t * ray.direction,
      m_normal,
      t
     );
  }

  initial_ray_sample_type sample_initial_ray(Random& random) const
  {
    const auto radius = std::sqrt(random.uniform(m_radius * m_radius));

    vector3_type u, v;
    std::tie(u, v) = orthonormal_basis(m_normal);

    real_type x, y;
    std::tie(x, y) = random.circle<real_type>();

    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = random.hemisphere_psa(u, v, m_normal);

    return initial_ray_sample_type(
      ray_type(m_center + (u * x + v * y) * radius, direction_o),
      m_normal
    );
  }
};

}
}
}
