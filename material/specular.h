#pragma once

#include <cmath>
#include "material/material.h"

namespace amber {
namespace material {

template <class Flux>
class Specular : public Material<Flux>
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
  flux_type m_reflectance;

public:
  Specular(const flux_type& r) :
    m_reflectance(r)
  {}

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
    const auto signed_cos_o = dot(ray.direction, hit.normal);
    const auto direction_i = ray.direction - 2 * signed_cos_o * hit.normal;

    return ray_sample_type(
      ray_type(hit.position, direction_i),
      m_reflectance / static_cast<real_type>(kEPS),
      1 / static_cast<real_type>(kEPS)
    );
  }
};

}
}
