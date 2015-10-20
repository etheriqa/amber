#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Lambertian : public Material<Radiant, RealType>
{
public:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

private:
  radiant_type m_kd;

public:
  explicit Lambertian(const radiant_type& kd) : m_kd(kd) {}

  bool is_emissive() const noexcept
  {
    return false;
  }

  SurfaceType surface_type() const noexcept
  {
    return SurfaceType::diffuse;
  }

  radiant_type emittance() const noexcept
  {
    return radiant_type();
  }

  radiant_type bsdf(const vector3_type& direction_i, const vector3_type& direction_o, const vector3_type& normal) const noexcept
  {
    if (dot(direction_i, normal) * dot(direction_o, normal) <= 0) {
      return radiant_type();
    } else {
      return m_kd / static_cast<real_type>(kPI);
    }
  }

  scattering_sample_type sample_scattering(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    const auto w = dot(direction_i, normal) > 0 ? normal : -normal;
    vector3_type direction_o;
    std::tie(direction_o, std::ignore) = random.hemisphere_psa(w);

    scattering_sample_type sample;
    sample.direction_o = direction_o;
    sample.bsdf = m_kd / static_cast<real_type>(kPI);
    sample.psa_probability = static_cast<real_type>(1 / kPI);
    return sample;
  }
};

}
}
