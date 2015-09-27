#pragma once

#include <cmath>
#include "material/material.h"

namespace amber {
namespace material {

template <class Flux>
class Refraction : public Material<Flux>
{
public:
  using material_type   = Material<Flux>;

  using flux_type       = typename material_type::flux_type;
  using hit_type        = typename material_type::hit_type;
  using ray_sample_type = typename material_type::ray_sample_type;
  using ray_type        = typename material_type::ray_type;
  using real_type       = typename material_type::real_type;
  using vector3_type    = typename material_type::vector3_type;

private:
  real_type m_refractive_index;

public:
  Refraction(real_type ri) : m_refractive_index(ri) {}

  bool is_emissive() const noexcept
  {
    return false;
  }

  bool is_specular() const noexcept
  {
    return true;
  }

  flux_type emittance() const noexcept
  {
    return flux_type();
  }

  flux_type bsdf(const vector3_type&, const vector3_type&, const vector3_type&) const noexcept
  {
    return flux_type();
  }

  ray_sample_type sample_ray_bsdf(const hit_type& hit, const ray_type& ray, Random& random) const
  {
    const auto signed_cos_i = dot(ray.direction, hit.normal);
    const ray_type reflection_ray(
      hit.position,
      ray.direction - 2 * signed_cos_i * hit.normal
    );

    const auto ri = signed_cos_i > 0 ? m_refractive_index : 1 / m_refractive_index;
    const auto square_cos_o = 1 - (1 - signed_cos_i * signed_cos_i) * (ri * ri);

    if (square_cos_o < 0) {
      // Full reflection
      return ray_sample_type(
        reflection_ray,
        flux_type(static_cast<real_type>(1 / kEPS)),
        static_cast<real_type>(1 / kEPS)
      );
    }

    const auto cos_i = std::abs(signed_cos_i);
    const auto cos_o = std::sqrt(square_cos_o);
    const auto p_reflection = fresnel(std::acos(cos_i), std::acos(cos_o));

    if (p_reflection > random.uniform<real_type>()) {
      // Partial reflection
      return ray_sample_type(
        reflection_ray,
        flux_type(static_cast<real_type>(1 / kEPS)) * p_reflection,
        static_cast<real_type>(1 / kEPS) * p_reflection
      );
    } else {
      // Refraction
      return ray_sample_type(
        ray_type(
          hit.position,
          ray.direction + (cos_i - cos_o / ri) * hit.normal
        ),
        flux_type(static_cast<real_type>(1 / kEPS)) * (1 - p_reflection),
        static_cast<real_type>(1 / kEPS) * (1 - p_reflection)
      );
    }
  }

  real_type fresnel(real_type alpha, real_type beta) const
  {
    const auto s = std::sin(alpha - beta) / std::sin(alpha + beta);
    const auto t = std::tan(alpha - beta) / std::tan(alpha + beta);
    return (s * s + t * t) / 2;
  }
};

}
}
