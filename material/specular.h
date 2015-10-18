#pragma once

#include <cmath>
#include "constant.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant>
class Specular : public Material<Radiant>
{
public:
  using material_type          = Material<Radiant>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

private:
  radiant_type m_ks;

public:
  explicit Specular(const radiant_type& ks) : m_ks(ks) {}

  bool is_emissive() const noexcept
  {
    return false;
  }

  SurfaceType surface_type() const noexcept
  {
    return SurfaceType::specular;
  }

  radiant_type emittance() const noexcept
  {
    return radiant_type();
  }

  radiant_type bsdf(const vector3_type&, const vector3_type&, const vector3_type&) const noexcept
  {
    return radiant_type();
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
