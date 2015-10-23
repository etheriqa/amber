#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Specular : public Material<Radiant, RealType> {
private:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

  radiant_type ks_;

public:
  explicit Specular(const radiant_type& ks) noexcept : ks_(ks) {}

  SurfaceType surfaceType() const noexcept { return SurfaceType::specular; }

  scattering_sample_type sampleScattering(const radiant_type&,
                                          const vector3_type& direction_i,
                                          const vector3_type& normal,
                                          Random& random) const {
    const auto direction_o =
      2 * dot(direction_i, normal) * normal - direction_i;
    scattering_sample_type sample;
    sample.direction_o = direction_o;
    sample.bsdf = ks_ / kEPS;
    sample.psa_probability = 1 / kEPS;
    return sample;
  }
};

}
}
