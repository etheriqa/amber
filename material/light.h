#pragma once

#include "constant.h"
#include "material/lambertian.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Flux>
class Light : public Material<Flux>
{
public:
  using material_type          = Material<Flux>;

  using flux_type              = typename material_type::flux_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

private:
  flux_type m_emittance;

public:
  explicit Light(const flux_type& e) : m_emittance(e) {}

  bool is_emissive() const noexcept
  {
    return true;
  }

  bool is_specular() const noexcept
  {
    return false;
  }

  flux_type emittance() const noexcept
  {
    return m_emittance;
  }

  flux_type bsdf(const vector3_type&, const vector3_type&, const vector3_type&) const noexcept
  {
    return flux_type();
  }

  scattering_sample_type sample_scattering(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    return Lambertian<flux_type>(flux_type()).sample_scattering(direction_i, normal, random);
  }
};

}
}
