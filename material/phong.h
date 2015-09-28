#pragma once

#include <cmath>
#include "constant.h"
#include "material/lambertian.h"
#include "material/material.h"

namespace amber {
namespace material {

template <class Flux>
class Phong : public Material<Flux>
{
public:
  using material_type = Material<Flux>;

  using flux_type       = typename material_type::flux_type;
  using hit_type        = typename material_type::hit_type;
  using ray_sample_type = typename material_type::ray_sample_type;
  using ray_type        = typename material_type::ray_type;
  using real_type       = typename material_type::real_type;
  using vector3_type    = typename material_type::vector3_type;

private:
  flux_type m_kd; // diffuse reflectivity
  flux_type m_ks; // specular reflectivity
  real_type m_n;  // specular exponent
  real_type m_p_diffuse;

public:
  Phong(const flux_type& kd, const flux_type& ks, real_type n) :
    m_kd(kd), m_ks(ks), m_n(n)
  {
    const auto diffuse_weight = m_kd.x + m_kd.y + m_kd.z;
    const auto specular_weight = m_ks.x + m_ks.y + m_ks.z;
    m_p_diffuse = diffuse_weight / (diffuse_weight + specular_weight);
  }

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
      const auto cos_alpha = dot(direction_o, 2 * dot(direction_i, normal) * normal - direction_i);
      return m_kd / static_cast<real_type>(kPI)
        + m_ks * (m_n + 2) / static_cast<real_type>(2 * kPI) * std::pow(std::max<real_type>(0, cos_alpha), m_n);
    }
  }

  ray_sample_type sample_ray_bsdf(const hit_type& hit, const ray_type& ray, Random& random) const
  {
    const auto direction = m_p_diffuse > random.uniform<real_type>()
      ? sample_diffuse_direction(hit, ray, random)
      : sample_specular_direction(hit, ray, random);
    const auto cos_alpha = dot(direction, ray.direction - 2 * dot(ray.direction, hit.normal) * hit.normal);

    const auto psa_probability = m_p_diffuse / static_cast<real_type>(kPI)
      + (1 - m_p_diffuse) * (m_n + 1) / static_cast<real_type>(2 * kPI) * std::pow(std::max<real_type>(0, cos_alpha), m_n);

    return ray_sample_type(
      ray_type(hit.position, direction),
      bsdf(-ray.direction, direction, hit.normal),
      psa_probability
    );
  }

private:
  vector3_type sample_diffuse_direction(const hit_type& hit, const ray_type& ray, Random& random) const
  {
    const auto signed_cos_i = dot(ray.direction, hit.normal);
    const auto normal = signed_cos_i < 0 ? hit.normal : -hit.normal;

    return std::get<0>(random.hemisphere_psa(normal));
  }

  vector3_type sample_specular_direction(const hit_type& hit, const ray_type& ray, Random& random) const
  {
    while (true) {
      const auto r0 = random.uniform<real_type>(), r1 = random.uniform<real_type>();
      const auto cos_alpha = std::pow(r0, 1 / (m_n + 1));
      const auto sin_alpha = std::sqrt(1 - std::pow(r0, 2 / (m_n + 1)));
      const auto phi = static_cast<real_type>(2 * kPI) * r1;

      const auto x = sin_alpha * std::cos(phi);
      const auto y = sin_alpha * std::sin(phi);
      const auto z = cos_alpha;

      vector3_type w = ray.direction - 2 * dot(ray.direction, hit.normal) * hit.normal;
      vector3_type u, v;
      std::tie(u, v) = orthonormal_basis(w);
      const auto direction = u * x + v * y + w * z;

      if (dot(ray.direction, hit.normal) * dot(direction, hit.normal) > 0) {
        continue;
      }

      return direction;
    }
  }
};

}
}
