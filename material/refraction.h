#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Refraction : public Material<Radiant, RealType> {
private:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

  real_type refractive_index_;

public:
  explicit Refraction(real_type refractive_index) noexcept
    : refractive_index_(refractive_index) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }

  scattering_sample_type sampleScattering(const radiant_type&,
                                          const vector3_type& direction_i,
                                          const vector3_type& normal,
                                          Random& random) const {
    const auto signed_cos_i = dot(direction_i, normal);
    const auto reflection_direction_o = 2 * signed_cos_i * normal - direction_i;

    const auto ri = signed_cos_i > 0
      ? 1 / refractive_index_
      : refractive_index_;
    const auto square_cos_o = 1 - (1 - signed_cos_i * signed_cos_i) * (ri * ri);

    if (square_cos_o < 0) {
      // Full reflection
      scattering_sample_type sample;
      sample.direction_o = reflection_direction_o;
      sample.bsdf = radiant_type(1 / kEPS);
      sample.psa_probability = 1 / kEPS;
      return sample;
    }

    const auto cos_i = std::abs(signed_cos_i);
    const auto cos_o = std::sqrt(square_cos_o);
    const auto p_reflection = fresnel(std::acos(cos_i), std::acos(cos_o));

    if (p_reflection > random.uniform<real_type>()) {
      // Partial reflection
      scattering_sample_type sample;
      sample.direction_o = reflection_direction_o;
      sample.bsdf = radiant_type(p_reflection / kEPS);
      sample.psa_probability = p_reflection / kEPS;
      return sample;
    } else {
      // Refraction
      scattering_sample_type sample;
      sample.direction_o = (cos_i - cos_o / ri) * normal - direction_i;
      sample.bsdf = radiant_type((1 - p_reflection) / kEPS);
      sample.psa_probability = (1 - p_reflection) / kEPS;
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
