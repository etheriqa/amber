#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <class Flux>
class Lambertian : public Material<Flux>
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
  flux_type m_kd;

public:
  explicit Lambertian(const flux_type& kd) : m_kd(kd) {}

  bool is_emissive() const noexcept
  {
    return false;
  }

  bool is_specular() const noexcept
  {
    return false;
  }

  flux_type emittance() const noexcept
  {
    return flux_type();
  }

  flux_type bsdf(const vector3_type& direction_i, const vector3_type& direction_o, const vector3_type& normal) const noexcept
  {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return flux_type();
    } else {
      return m_kd / static_cast<real_type>(kPI);
    }
  }

  ray_sample_type sample_ray_bsdf(const hit_type& hit, const ray_type& ray, Random& random) const
  {
    const auto signed_cos_i = dot(ray.direction, hit.normal);
    const auto normal = signed_cos_i < 0 ? hit.normal : -hit.normal;

    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = random.hemisphere_psa(normal);

    return ray_sample_type(
      ray_type(hit.position, direction_o),
      m_kd / static_cast<real_type>(kPI),
      static_cast<real_type>(1 / kPI)
    );
  }
};

}
}
