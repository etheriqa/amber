#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <class Flux>
class Specular : public Material<Flux>
{
public:
  using material_type          = Material<Flux>;

  using flux_type              = typename material_type::flux_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

private:
  flux_type m_ks;

public:
  explicit Specular(const flux_type& ks) : m_ks(ks) {}

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
    const auto direction_o = 2 * dot(direction_i, normal) * normal - direction_i;

    scattering_sample_type sample;
    sample.direction_o = direction_o;
    sample.bsdf = m_ks / static_cast<real_type>(kEPS);
    sample.psa_probability = static_cast<real_type>(1 / kEPS);
    return sample;
  }
};

}
}
