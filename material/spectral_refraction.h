#pragma once

#include "material/refraction.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class SpectralRefraction : public Material<Radiant, RealType> {
private:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

  radiant_type refractive_indexes_;

public:
  explicit SpectralRefraction(const radiant_type& refractive_indexes) noexcept
    : refractive_indexes_(refractive_indexes) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }

  scattering_sample_type sampleScattering(const radiant_type& radiant,
                                          const vector3_type& direction_i,
                                          const vector3_type& normal,
                                          Random& random) const {
    const auto u = random.uniform<real_type>(radiant.sum());
    radiant_type filter;
    real_type ri;
    if (u < radiant.r()) {
      filter = radiant_type(1, 0, 0);
      ri = refractive_indexes_.r();
    } else if (u < radiant.r() + radiant.g()) {
      filter = radiant_type(0, 1, 0);
      ri = refractive_indexes_.g();
    } else {
      filter = radiant_type(0, 0, 1);
      ri = refractive_indexes_.b();
    }
    auto sample = Refraction<radiant_type, real_type>(ri)
      .sampleScattering(radiant_type(), direction_i, normal, random);
    sample.bsdf *= filter;
    sample.psa_probability *= (radiant * filter).sum() / radiant.sum();
    return sample;
  }
};

}
}
