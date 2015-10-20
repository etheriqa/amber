#pragma once

#include "material/refraction.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class SpectralRefraction : public Material<Radiant, RealType> {
public:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

private:
  radiant_type refractive_indexes_;

public:
  explicit SpectralRefraction(const radiant_type& refractive_indexes) :
    refractive_indexes_(refractive_indexes) {}

  bool is_emissive() const noexcept { return false; }
  SurfaceType surface_type() const noexcept { return SurfaceType::specular; }
  radiant_type emittance() const noexcept { return radiant_type(); }
  radiant_type bsdf(const vector3_type&,
                    const vector3_type&,
                    const vector3_type&) const noexcept {
    return radiant_type();
  }

  scattering_sample_type sample_scattering(const radiant_type& radiant,
                                           const vector3_type& direction_i,
                                           const vector3_type& normal,
                                           Random& random) const {
    const auto u = random.uniform<real_type>(radiant.sum());
    radiant_type filter;
    real_type refractive_index, probability;
    if (u < radiant.r()) {
      filter = radiant_type(1, 0, 0);
      refractive_index = refractive_indexes_.r();
      probability = radiant.r() / radiant.sum();
    } else if (u < radiant.r() + radiant.g()) {
      filter = radiant_type(0, 1, 0);
      refractive_index = refractive_indexes_.g();
      probability = radiant.g() / radiant.sum();
    } else {
      filter = radiant_type(0, 0, 1);
      refractive_index = refractive_indexes_.b();
      probability = radiant.b() / radiant.sum();
    }
    const auto refraction =
      Refraction<radiant_type, real_type>(refractive_index);
    auto sample =
      refraction.sample_scattering(radiant_type(), direction_i, normal, random);
    sample.bsdf *= filter;
    sample.psa_probability *= probability;
    return sample;
  }
};

}
}
