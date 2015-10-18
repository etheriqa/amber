#pragma once

#include <cmath>
#include "constant.h"
#include "material/lambertian.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Flux>
class Phong : public Material<Flux>
{
public:
  using material_type          = Material<Flux>;

  using flux_type              = typename material_type::flux_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

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

  SurfaceType surface_type() const noexcept
  {
    return SurfaceType::diffuse;
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

  scattering_sample_type sample_scattering(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    const auto direction_o = m_p_diffuse > random.uniform<real_type>()
      ? sample_diffuse_direction(direction_i, normal, random)
      : sample_specular_direction(direction_i, normal, random);
    const auto cos_alpha = dot(direction_o, 2 * dot(direction_i, normal) * normal - direction_i);

    const auto psa_probability = m_p_diffuse / static_cast<real_type>(kPI)
      + (1 - m_p_diffuse) * (m_n + 1) / static_cast<real_type>(2 * kPI) * std::pow(std::max<real_type>(0, cos_alpha), m_n);

    scattering_sample_type sample;
    sample.direction_o = direction_o;
    sample.bsdf = bsdf(direction_i, direction_o, normal);
    sample.psa_probability = psa_probability;
    return sample;
  }

private:
  vector3_type sample_diffuse_direction(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    const auto w = dot(direction_i, normal) > 0 ? normal : -normal;
    return std::get<0>(random.hemisphere_psa(w));
  }

  vector3_type sample_specular_direction(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    while (true) {
      const auto r0 = random.uniform<real_type>(), r1 = random.uniform<real_type>();
      const auto cos_alpha = std::pow(r0, 1 / (m_n + 1));
      const auto sin_alpha = std::sqrt(1 - std::pow(r0, 2 / (m_n + 1)));
      const auto phi = static_cast<real_type>(2 * kPI) * r1;

      const auto x = sin_alpha * std::cos(phi);
      const auto y = sin_alpha * std::sin(phi);
      const auto z = cos_alpha;

      vector3_type w = 2 * dot(direction_i, normal) * normal - direction_i;
      vector3_type u, v;
      std::tie(u, v) = orthonormal_basis(w);
      const auto direction_o = u * x + v * y + w * z;

      if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
        continue;
      }

      return direction_o;
    }
  }
};

}
}
