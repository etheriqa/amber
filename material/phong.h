#pragma once

#include <cmath>
#include "constant.h"
#include "material/lambertian.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Phong : public Material<Radiant, RealType> {
private:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

  radiant_type kd_; // diffuse reflectivity
  radiant_type ks_; // specular reflectivity
  real_type n_;     // specular exponent
  real_type p_diffuse_;

public:
  Phong(const radiant_type& kd, const radiant_type& ks, real_type n) noexcept
    : kd_(kd), ks_(ks), n_(n), p_diffuse_(kd.sum() / (kd.sum() + ks.sum())) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::diffuse; }

  radiant_type bsdf(const vector3_type& direction_i,
                    const vector3_type& direction_o,
                    const vector3_type& normal) const noexcept {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return radiant_type();
    }
    const auto cos_alpha =
      dot(direction_o, 2 * dot(direction_i, normal) * normal - direction_i);
    return kd_ / kPI
      + ks_ * (n_ + 2) / (2 * kPI)
      * std::pow(std::max<real_type>(0, cos_alpha), n_);
  }

  scattering_sample_type sampleScattering(const radiant_type&,
                                          const vector3_type& direction_i,
                                          const vector3_type& normal,
                                          Random& random) const {
    const auto direction_o = p_diffuse_ > random.uniform<real_type>()
      ? sampleDirectionFromDiffuseComponent(direction_i, normal, random)
      : sampleDirectionFromSpecularComponent(direction_i, normal, random);
    const auto cos_alpha =
      dot(direction_o, 2 * dot(direction_i, normal) * normal - direction_i);
    const auto psa_probability =
      p_diffuse_ / kPI
      + (1 - p_diffuse_) * (n_ + 1) / (2 * kPI)
      * std::pow(std::max<real_type>(0, cos_alpha), n_);
    scattering_sample_type sample;
    sample.direction_o = direction_o;
    sample.bsdf = bsdf(direction_i, direction_o, normal);
    sample.psa_probability = psa_probability;
    return sample;
  }

private:
  vector3_type
  sampleDirectionFromDiffuseComponent(const vector3_type& direction_i,
                                      const vector3_type& normal,
                                      Random& random) const {
    const auto w = dot(direction_i, normal) > 0 ? normal : -normal;
    return std::get<0>(random.hemisphere_psa(w));
  }

  vector3_type
  sampleDirectionFromSpecularComponent(const vector3_type& direction_i,
                                       const vector3_type& normal,
                                       Random& random) const {
    while (true) {
      const auto r0 = random.uniform<real_type>();
      const auto r1 = random.uniform<real_type>();
      const auto cos_alpha = std::pow(r0, 1 / (n_ + 1));
      const auto sin_alpha = std::sqrt(1 - std::pow(r0, 2 / (n_ + 1)));
      const auto phi = 2 * kPI * r1;

      const auto x = sin_alpha * std::cos(phi);
      const auto y = sin_alpha * std::sin(phi);
      const auto z = cos_alpha;

      vector3_type w = 2 * dot(direction_i, normal) * normal - direction_i;
      vector3_type u, v;
      std::tie(u, v) = orthonormalBasis(w);
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
