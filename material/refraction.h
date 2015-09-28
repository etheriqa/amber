#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <class Flux>
class Refraction : public Material<Flux>
{
public:
  using material_type          = Material<Flux>;

  using flux_type              = typename material_type::flux_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

private:
  real_type m_refractive_index;

public:
  explicit Refraction(real_type ri) : m_refractive_index(ri) {}

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

  scattering_sample_type sample_scattering(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    const auto signed_cos_i = dot(direction_i, normal);
    const auto reflection_direction_o = 2 * signed_cos_i * normal - direction_i;

    const auto ri = signed_cos_i > 0 ? 1 / m_refractive_index : m_refractive_index;
    const auto square_cos_o = 1 - (1 - signed_cos_i * signed_cos_i) * (ri * ri);

    if (square_cos_o < 0) {
      // Full reflection
      scattering_sample_type sample;
      sample.direction_o = reflection_direction_o;
      sample.bsdf = flux_type(static_cast<real_type>(1 / kEPS));
      sample.psa_probability = static_cast<real_type>(1 / kEPS);
      return sample;
    }

    const auto cos_i = std::abs(signed_cos_i);
    const auto cos_o = std::sqrt(square_cos_o);
    const auto p_reflection = fresnel(std::acos(cos_i), std::acos(cos_o));

    if (p_reflection > random.uniform<real_type>()) {
      // Partial reflection
      scattering_sample_type sample;
      sample.direction_o = reflection_direction_o;
      sample.bsdf = flux_type(static_cast<real_type>(1 / kEPS)) * p_reflection;
      sample.psa_probability = static_cast<real_type>(1 / kEPS) * p_reflection;
      return sample;
    } else {
      // Refraction
      scattering_sample_type sample;
      sample.direction_o = (cos_i - cos_o / ri) * normal - direction_i;
      sample.bsdf = flux_type(static_cast<real_type>(1 / kEPS)) * (1 - p_reflection);
      sample.psa_probability = static_cast<real_type>(1 / kEPS) * (1 - p_reflection);
      return sample;
    }
  }

private:
  static real_type fresnel(real_type alpha, real_type beta) noexcept
  {
    const auto s = std::sin(alpha - beta) / std::sin(alpha + beta);
    const auto t = std::tan(alpha - beta) / std::tan(alpha + beta);
    return (s * s + t * t) / 2;
  }
};

}
}
